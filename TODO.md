# ToDo

Open work items. The details are in the linked plans.

- [x] **UM980 / UM982: find the cause of the 1 s BESTNAVB gap.** Solved 2026-11-12: it is a start-up effect (one BESTNAVB on the first whole second, then a 1 s pause before 20 Hz). A 10 min capture on the UM980 lost no epoch in steady state, with 0 CRC errors. Not re-checked on the UM982. See [plans/gnss-um98x-config.md](plans/gnss-um98x-config.md), "Bench results (2026-11-12, UM980 gap capture: the 1 s BESTNAVB gap)".
