[README.md](https://github.com/user-attachments/files/33273960/README.md)
# Trash-Track# SL37 — Intelligent Waste Collection Scheduler
### QNX eHACK 2026 | Team: Deadline Drivers

---

## Problem Statement

Urban waste collection today runs on **fixed schedules and fixed routes** — the truck visits every bin on the same street at the same time, every day, whether the bin is empty or overflowing.

This causes three real-world failures:

| Problem | Impact |
|---|---|
| Bins overflow before collection day | Public health hazard, bad smell, pests |
| Trucks visit nearly-empty bins | Fuel waste, unnecessary traffic, carbon cost |
| No emergency response to a sudden overflow | Bin stays blocked for hours until the next scheduled visit |

**The core gap:** There is no real-time feedback loop between bin fill status and vehicle routing. The truck drives blind.

---

## Our Solution

**A QNX RTOS-based Intelligent Waste Collection Scheduler** that:

1. **Monitors** each bin's fill level continuously (every 1 second)
2. **Scores and ranks** bins dynamically using a weighted algorithm
3. **Routes** the collection vehicle to the highest-priority bin first
4. **Reacts immediately** when any bin crosses the 90% overflow threshold — without waiting for the next scheduled update

The key innovation is the **Emergency Override** — a real-time, priority-driven interrupt that fires the moment a bin overflows and immediately reroutes the vehicle. This is not a software workaround; it is a hard real-time guarantee made possible by QNX's deterministic scheduling.

---

## Why QNX RTOS?

Normal operating systems (Linux, Windows) cannot guarantee **when** a task will run. Under load, a high-priority emergency notification might be delayed by milliseconds or even seconds.

QNX RTOS solves this with:

- **SCHED_FIFO scheduling** — the highest-priority ready thread always runs immediately
- **Native IPC via channels and pulses** — zero-copy, microsecond-latency event delivery
- **Priority inheritance** — no priority inversion, no deadlocks
- **Deterministic response time** — the emergency task response is bounded, not best-effort

This project would not be safe to deploy on a general-purpose OS. On QNX, the overflow response time is provably bounded.

---

## System Architecture

```
                        ONE QNX PROCESS
                              │
         ┌────────────────────┼──────────────────────┐
         │                    │                       │
         ▼                    ▼                       ▼
  Emergency Task        Route Scheduler         Vehicle Task
  Priority 25           Priority 20             Priority 10
  (HIGHEST)                                     (LOWEST)
         ▲                    ▲
         │                    │
  ┌──────┴──────┐    ┌────────┴────────┐
  │  Bin Mon 0  │    │   Bin Mon 1     │  ...  Bin Mon 3
  │  Priority 15│    │   Priority 15   │       Priority 15
  └─────────────┘    └─────────────────┘
```

**8 threads total** inside a single process:
- 1 × Emergency Task
- 1 × Route Scheduler
- 4 × Bin Monitor (one per street)
- 1 × Vehicle Task
- 1 × Main thread (keeps process alive)

---

## How It Works — Step by Step

### Normal Operation (every second)

```
Bin Monitor wakes every 1000 ms
        │
        ▼
Random person arrives (30% chance per cycle)
        │
        ├── Bin has space → accept dump → fill level increases
        └── Bin LOCKED   → wait up to 2 sec (condition variable)
                                │
                          Vehicle empties bin
                          → broadcast wakes waiter
                          → dump accepted
        │
        ▼
Send BIN_READING_PULSE to Emergency Task
```

### Route Scheduling (every 5 seconds)

```
POSIX Timer fires → ROUTE_TIMER_PULSE
        │
        ▼
Route Scheduler wakes
        │
        ▼
Score each bin:
  score = 0.6 × fill_level
        + 0.3 × (1 / distance_km)
        + 0.1 × wait_hours
        │
        ▼
qsort() — highest score first
Overflowed bins always go to position 1
        │
        ▼
Print updated route:
  1. Street 2 (fill=88%)
  2. Street 0 (fill=74%)
  3. Street 3 (fill=45%)
  4. Street 1 (fill=22%)
```

### Emergency Override (instant, any time)

```
Bin Monitor sends BIN_READING_PULSE (fill = 92%)
        │
        ▼
Emergency Task (Priority 25) preempts everything
        │
        ▼
Arm 300 ms watchdog timer
        │
        ▼
Lock bin door → DOOR_LOCKED (no more dumping)
Mark overflow flag → overflowed = 1
        │
        ▼
Send OVERRIDE_PULSE to Route Scheduler
        │
        ▼
Cancel watchdog (response done in < 5 ms)
Log: "Override response time: X ms"
        │
        ▼
Route Scheduler immediately recomputes
Overflowed bin → position 1 in route
```

---

## Route Scoring Formula

```
score = 0.6 × fill_level
      + 0.3 × (1 / distance_km)
      + 0.1 × wait_hours
```

| Factor | Weight | Why |
|---|---|---|
| Fill level | 60% | The most critical urgency signal |
| 1 / distance | 30% | Closer bins are cheaper to collect first |
| Wait time | 10% | Bins waiting longer get a small priority boost |

Overflowed bins (`fill ≥ 90%`) **always sort first**, regardless of score.

---

## IPC Architecture

```
SENDER                  PULSE               RECEIVER
──────────────────────────────────────────────────────
Bin Monitor    →  BIN_READING_PULSE   →  Emergency Task
POSIX Timer    →  ROUTE_TIMER_PULSE   →  Route Scheduler
Emergency Task →  OVERRIDE_PULSE      →  Route Scheduler
Watchdog Timer →  WATCHDOG_PULSE      →  Emergency Task
```

All communication uses **QNX native channels and pulses**:
- `ChannelCreate()` — receiver creates a mailbox
- `ConnectAttach()` — sender gets a connection ID
- `MsgSendPulse()` — sender fires a pulse (non-blocking)
- `MsgReceive()` — receiver blocks until a pulse arrives

---

## Synchronization

| Mechanism | Protects | Used by |
|---|---|---|
| `bin_map_mutex` | `bin_map[]` — all bin data | Bin Monitor, Emergency Task, Vehicle Task |
| `route_mutex` | `route[]` — ordered street list | Route Scheduler (write), Vehicle Task (read) |
| `log_mutex` | Console + log file output | All threads |
| `space_cond` | Condition variable for full-bin wait | Bin Monitor (wait), Vehicle Task (broadcast) |

The **condition variable** (`space_cond`) is important: when a person arrives at a locked bin, the Bin Monitor thread sleeps efficiently (no CPU spin) until the Vehicle Task empties the bin and broadcasts. It uses a `while` loop (not `if`) to handle spurious wakeups correctly.

---

## Watchdog Timer

The Emergency Task arms a **300 ms watchdog** before sending the override pulse:

```
Watchdog armed (300 ms countdown)
        │
        ├── Override completed in time → cancel watchdog ✅
        └── 300 ms expires first       → WATCHDOG_PULSE fires
                                          "WATCHDOG ESCALATION" logged ❌
```

This guarantees the emergency response is **time-bounded**. If the system is overloaded and misses the deadline, it is detected and logged. No silent failures.

---

## Door State Machine

Each bin has a door with three states:

```
  DOOR_IDLE
      │
      │  Person arrives, bin has space
      ▼
  DOOR_OPEN       ← fill level increases during dump
      │
      │  Dump complete
      ▼
  DOOR_IDLE       ← resets automatically

  BUT if fill ≥ 90%:
  DOOR_IDLE ──→ DOOR_LOCKED   (Emergency Task locks it)
      ▲               │
      │               │  Vehicle empties bin
      └───────────────┘  (Vehicle Task resets to IDLE)
                          pthread_cond_broadcast() wakes any waiters
```

---

## Key Numbers

| Parameter | Value |
|---|---|
| Number of bins (streets) | 4 |
| Overflow threshold | 90% fill |
| Bin Monitor update period | 1 second |
| Normal route recalculation | Every 5 seconds |
| Vehicle service cycle | Every 3 seconds |
| Emergency watchdog deadline | 300 ms |
| Achieved override response time | < 5 ms |
| Thread count | 8 |
| Process count | 1 |
| CPU scheduling policy | SCHED_FIFO |

---

## File Structure

```
waste_scheduler.c    ← complete single-file simulation
waste_scheduler.log  ← generated at runtime (timestamped events)
README.md            ← this file
```

---

## Build & Run

```bash
# Build (QNX Momentics or command line)
qcc -Vgcc_ntox86_64 -lm -o waste_scheduler waste_scheduler.c

# Run
./waste_scheduler
```

**Expected output:**

```
=====================================================
 QNX eHACK 2026 - PS37 Intelligent Waste Collection
 Team: Deadline Drivers
=====================================================

[1234 ms] Emergency Task started (priority 25, threshold 90%)
[1235 ms] Route Scheduler started (priority 20, period 5s)
[1436 ms] Bin Monitor 0 started (priority 15)
...
  Street 2: dump accepted, fill now 67.0%
  Street 2: dump accepted, fill now 81.0%
  Street 2: dump accepted, fill now 92.0%
[5821 ms] OVERFLOW: Street 2 at 92.0% - door LOCKED
[5822 ms] Override response time: 3.4 ms (target <= 300ms)

--- ROUTE UPDATED ---
  1. Street 2  (fill=92.0%, OVERFLOW)
  2. Street 0  (fill=74.0%)
  3. Street 3  (fill=45.0%)
  4. Street 1  (fill=22.0%)
---------------------
```

---

## RTOS Concepts Demonstrated

| Concept | Implementation |
|---|---|
| Real-time scheduling | SCHED_FIFO with 4 distinct priority levels |
| Preemption | Emergency Task (P25) preempts Bin Monitor (P15) instantly |
| IPC | QNX channels + pulses (4 pulse types) |
| Shared memory protection | 3 POSIX mutexes |
| Event-driven synchronization | Condition variable (space_cond) |
| Periodic tasks | POSIX timer → ROUTE_TIMER_PULSE every 5 s |
| Watchdog | 300 ms deadline timer on emergency path |
| Fault detection | Watchdog escalation log on missed deadline |
| State machine | Door FSM: IDLE / OPEN / LOCKED |
| Greedy algorithm | Weighted-score route ordering with qsort() |

---

## Team

**Team Name:** Deadline Drivers  
**Hackathon:** QNX eHACK 2026  
**Problem Statement:** SL37 — Intelligent Waste Collection Scheduler

---

*Built for QNX eHACK 2026. Software simulation only — no hardware prototype required per organizer confirmation.*
