#ifndef __CLKSOURCE_TIMER_SP804_H
#define __CLKSOURCE_TIMER_SP804_H

struct clk;

void __sp804_clocksource_and_sched_clock_init(void __iomem *,
					      const char *,
					      struct clk *, bool);
void __sp804_clockevents_init(void __iomem *, unsigned int,
			      struct clk *, const char *);
void sp804_timer_disable(void __iomem *);

static inline void sp804_clocksource_init(void __iomem *base, const char *name)
{
	__sp804_clocksource_and_sched_clock_init(base, name, NULL, false);
}

static inline void sp804_clocksource_and_sched_clock_init(void __iomem *base,
							  const char *name)
{
	__sp804_clocksource_and_sched_clock_init(base, name, NULL, true);
}

static inline void sp804_clockevents_init(void __iomem *base, unsigned int irq, const char *name)
{
	__sp804_clockevents_init(base, irq, NULL, name);

}
#endif
