/*
 *
 * Dummy load 
 *
 * Author  : Per Hallsmark <per.hallsmark@windriver.com>
 *
 * License : GPLv2
 *
 * Using PID regulator, try tuning into a busy loop that
 * generates a requested percentage load.
 *
 * The actual load is not exact, but close to, as requested up to about 95%.
 * If it is way far off the pid constants might need tuning for this system.
 * Around 50% load, the pid controller gets a bit nervous. Most probably some
 * pid constants tuning todo...
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <errno.h>
#include <pthread.h>
#include <sys/resource.h>

#define CLOCK CLOCK_MONOTONIC


static float requested_load;
static float actual_load;
static int ticks_per_second;
static int us_per_tick;

static struct rusage lru;

#define ILIMIT 10000.0

static float dtns;
static float e = 0.0;
static float Kp = 200000.0;
static float p  = 0.0;
static float Ki = 500.0;
static float itmp = 0.0;
static float imin = -1.0 * ILIMIT;
static float imax = 1.0 * ILIMIT;
static float i  = 0.0;
static float Kd = 2.0;
static float d  = 0.0;
static float dtmp  = 0.0;
static float pp = 0.0;
static float out = 0.0;
static float outmax = 100000000.0;
static float outmin = 0.0;
static float lastout = 0.0;

static uint64_t looplimit;

static void ts_add(const struct timespec *t1, const struct timespec *t2, struct timespec *t)
{
	t->tv_sec = t1->tv_sec + t2->tv_sec;
	t->tv_nsec = t1->tv_nsec + t2->tv_nsec;
	if (t->tv_nsec >= 1000000000L) {
		t->tv_sec++ ;
		t->tv_nsec -= 1000000000L;
	}
}

static void ts_sub(struct timespec *t1, struct timespec *t2, struct timespec *result)
{
	result->tv_nsec = t2->tv_nsec - t1->tv_nsec;
	if (result->tv_nsec < 0 ) {
		result->tv_nsec += 1000000000;
		result->tv_sec = t2->tv_sec - t1->tv_sec - 1;
	} else {
		result->tv_sec = t2->tv_sec - t1->tv_sec;
	}
}

/*
 * helper calling a function at a specifid periodic rate, which should be below 1 sec.
 * work time is expected to be guaranteed less than (period time - clock setup time)
 */
static void periodic_loop(const struct timespec *period, void (*work)(struct timespec *))
{
	struct timespec tbase;
	struct timespec tbefore, tafter, tactual;
	int status;

	clock_gettime(CLOCK, &tbase);

	while (1) {
		// calculate start of next period
		ts_add(&tbase, period, &tbase);

		clock_gettime(CLOCK, &tbefore);
		// wait for start of next period
		status = clock_nanosleep(CLOCK, TIMER_ABSTIME, &tbase, NULL);
		clock_gettime(CLOCK, &tafter);
		if (status)
			printf("status = %d\n", status);

		// execute loop work load
		ts_sub(&tbefore, &tafter, &tactual);
		if ((tactual.tv_sec == 0) && (tactual.tv_nsec < 1000000)) {
			// disregard of corner case were we calculated start of next period,
			// got interrupted and that for quite some time so that when kernel
			// get time to schedule this in, we are already past our period!
			continue;
		}
		work(&tactual);
	}
}

static void gen_load(struct timespec *tactual)
{
	static unsigned int volatile dummy;
	int i;

	// here we do the actual dummy work
	if (requested_load >= 100.0) {
		while (1) {
			dummy = (int)requested_load;
		}
	} else {
		for (i=0; i<looplimit; i++) {
			dummy = i;
		}
	}
}

static void *dummy_load(void *arg)
{
	int status;
	struct sched_param threadsched;
	struct timespec period;

	threadsched.sched_priority = 36;
	status = sched_setscheduler(0, SCHED_FIFO, &threadsched);
	if (status) {
		printf("%s: sched_setscheduler %d\n", __FUNCTION__, status);
		pthread_exit(&status);
	}

	period.tv_sec = 0;
	period.tv_nsec = (uint64_t)dtns;
	printf("period for dummy load is set to %ld s %ld ns\n", period.tv_sec, period.tv_nsec);

	periodic_loop(&period, gen_load);
}
	
static void error(void)
{
	printf("please enter dummy load as arg1, expressed in percentage.\n");
	exit(-1);
}

/*
 * This is similar to how procps package cmd "top" calculate load
 */
static void calc_load(uint64_t deltaus)
{
	int status;
	struct rusage cru;
	uint64_t delta_utime, delta_stime;
	static int updated = 0;
	

	memset(&cru, 0, sizeof(cru));

        status = getrusage(RUSAGE_SELF, &cru);
	if (status != 0)
		return;

	delta_utime = (cru.ru_utime.tv_sec*1000000 + cru.ru_utime.tv_usec) -
			(lru.ru_utime.tv_sec*1000000 + lru.ru_utime.tv_usec);

	delta_stime = (cru.ru_stime.tv_sec*1000000 + cru.ru_stime.tv_usec) -
			(lru.ru_stime.tv_sec*1000000 + lru.ru_stime.tv_usec);

	if (updated) {
		actual_load = 100.0 * ((float)((delta_utime+delta_stime) / (float)deltaus));
	}
	
	memcpy(&lru, &cru, sizeof(struct rusage));
	updated = 1;
}

static void tune_load()
{
	uint64_t myload;

	e = requested_load - actual_load;

	p = Kp*e;

	itmp += e;
	if (itmp > imax)
		itmp = imax;
	else if (itmp < imin)
		itmp = imin;
	i = Ki*itmp;

	d = Kd * (dtmp - e);
	dtmp = e;

	out = lastout + (p + i + d);

	if (out > outmax)
		out = outmax;
	else if (out < outmin)
		out = outmin;

	lastout = out;

	// export to dummyload thread
	looplimit = (uint64_t)out;
}

static void worker(struct timespec *tactual)
{
	uint64_t deltaus;

	deltaus = tactual->tv_sec*1000000+tactual->tv_nsec/1000;

	// calculate our load
	calc_load(deltaus);

	// adjust it for our needs
	tune_load();
}

int main(int argc, char *argv[])
{
	int status;
	struct timespec resolution;
	struct sched_param threadsched;
	pthread_t t;
	struct timespec period;

	if (argc != 2)
		error();

	requested_load = atof(argv[1]);

	if ((requested_load > 100.0) || (requested_load < 0.0))
		error();

	ticks_per_second = sysconf(_SC_CLK_TCK);
	us_per_tick = 1000000/ticks_per_second;

	clock_getres(CLOCK, &resolution);
	printf("clock resolution is %ld s %ld ns\n", resolution.tv_sec, resolution.tv_nsec);

	// set our period time to 1 tick period
	dtns = 1*us_per_tick*1000;

	period.tv_sec = 0;
	period.tv_nsec = (uint64_t)dtns;
	printf("period for pid regulator is set to %ld s %ld ns\n", period.tv_sec, period.tv_nsec);

	status = pthread_create(&t, NULL, dummy_load, NULL);

	threadsched.sched_priority = 37;
	status = sched_setscheduler(0, SCHED_FIFO, &threadsched);
	if (status) {
		printf("%s: sched_setscheduler %d\n", __FUNCTION__, status);
		pthread_exit(&status);
	}

	periodic_loop(&period, worker);
}
