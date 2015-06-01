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
#include <sys/syscall.h>
#include <sys/types.h>
#include <sys/time.h>

#ifndef SYS_gettid
#error "SYS_gettid unavailable on this system"
#endif

#define CLOCK CLOCK_MONOTONIC
#define MAX_INSTANCES 100

static float requested_load;
static int ticks_per_second;
static int us_per_tick;
static int instances;
static pid_t pid;

static struct sload {
	pid_t tid;
	struct timespec period;
	uint64_t periodns;
	struct rusage ru;
	struct rusage lru;
	float actual_load;
	float e;
	float p;
	float pp;
	float i;
	float itmp;
	float d;
	float dtmp;
	float out;
	float lastout;
	uint64_t looplimit;
	int updated;
} load[MAX_INSTANCES];

#define ILIMIT 10000.0

static float dtns;
static float Kp = 200000.0;
static float Ki = 500.0;
static float imin = -1.0 * ILIMIT;
static float imax = 1.0 * ILIMIT;
static float Kd = 2.0;
static float outmax = 100000000.0;
static float outmin = 0.0;

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
static void periodic_loop(const struct timespec *period, void (*work)(struct timespec *, struct timespec *, struct sload *l), struct sload *l)
{
	struct timespec tbase;
	struct timespec tbefore, tafter, tdelta;
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
		ts_sub(&tbefore, &tafter, &tdelta);
		if ((tdelta.tv_sec == 0) && (tdelta.tv_nsec < 1000000)) {
			// disregard of corner case were we calculated start of next period,
			// got interrupted and that for quite some time so that when kernel
			// get time to schedule this in, we are already past our period!
			l->updated = 0;
			continue;
		}
		work(&tbefore, &tdelta, l);
	}
}

static int safeguard_beyond_period(struct timespec *ts, uint64_t periodns_slip_max)
{
	struct timespec tnow, tdelta;
	uint64_t deltans;

	// disregard of corner case were we calculated duration of last period,
	// got interrupted and that for quite some time so that when kernel
	// get time to schedule this in, we will not have relevant data!
	clock_gettime(CLOCK, &tnow);
	ts_sub(ts, &tnow, &tdelta);
	deltans = tdelta.tv_sec*1000000000+tdelta.tv_nsec;
	if (deltans > periodns_slip_max) {
		printf("load avg not calculated, this process slipped too much\n");
		return 1;
	} else {
		return 0;
	}
}

static void gen_load(struct timespec *ts, struct timespec *tdelta, struct sload *l)
{
	static unsigned int volatile dummy;
	int i;
	int status;

	// here we do the delta dummy work
	if (requested_load >= 100.0) {
		while (1) {
			dummy = (int)requested_load;
		}
	} else {
		for (i=0; i<l->looplimit; i++) {
			dummy = i;
		}
	}
	status = getrusage(RUSAGE_THREAD, &(l->ru));
	if (status)
		perror("getrusage");
}

static void *dummy_load(void *arg)
{
	int status;
	struct sched_param threadsched;
	struct sload *l = (struct sload *)arg;

	threadsched.sched_priority = 36;
	status = sched_setscheduler(0, SCHED_FIFO, &threadsched);
	if (status) {
		printf("%s: sched_setscheduler %d\n", __FUNCTION__, status);
		pthread_exit(&status);
	}

	//load[instance].tid = gettid();
	l->tid = syscall(SYS_gettid);

	l->period.tv_sec = 0;
	l->period.tv_nsec = (uint64_t)dtns;
	printf("period for dummy load %d is set to %ld s %ld ns\n",
		l->tid, l->period.tv_sec, l->period.tv_nsec);

	l->periodns = l->period.tv_sec*1000000000+l->period.tv_nsec;

	periodic_loop(&(l->period), gen_load, l);
}
	
static void error(void)
{
	printf("please enter dummy load as arg1, expressed in percentage.\n");
	exit(-1);
}

/*
 * This is similar to how procps package cmd "top" calculate load
 */
static void calc_load(struct timespec *ts, uint64_t deltaus, struct sload *l)
{
	int status;
	struct rusage cru;
	uint64_t delta_utime, delta_stime, deltans;
	
	memcpy(&cru, &(l->ru), sizeof(struct rusage));

	delta_utime = (cru.ru_utime.tv_sec*1000000 + cru.ru_utime.tv_usec) -
			(l->lru.ru_utime.tv_sec*1000000 + l->lru.ru_utime.tv_usec);

	delta_stime = (cru.ru_stime.tv_sec*1000000 + cru.ru_stime.tv_usec) -
			(l->lru.ru_stime.tv_sec*1000000 + l->lru.ru_stime.tv_usec);

	if (safeguard_beyond_period(ts, (uint64_t)(l->periodns * 1.1))) {
		l->updated=0;
	} else {
		l->actual_load = 100.0 * ((float)((delta_utime+delta_stime) / (float)deltaus));
	}
	
	memcpy(&(l->lru), &cru, sizeof(struct rusage));
	l->updated = 1;
}

static void tune_load(struct timespec *ts, struct sload *l)
{
	uint64_t myload;

	l->e = requested_load - l->actual_load;

	l->p = Kp*l->e;

	l->itmp += l->e;
	if (l->itmp > imax)
		l->itmp = imax;
	else if (l->itmp < imin)
		l->itmp = imin;
	l->i = Ki*l->itmp;

	l->d = Kd * (l->dtmp - l->e);
	l->dtmp = l->e;

	l->out = l->lastout + (l->p + l->i + l->d);

	if (l->out > outmax)
		l->out = outmax;
	else if (l->out < outmin)
		l->out = outmin;

	l->lastout = l->out;

	if (safeguard_beyond_period(ts, (uint64_t)(l->periodns * 1.1))) {
		l->updated=0;
	} else {
		l->looplimit = (uint64_t)(l->out);
	}
}

static void worker(struct timespec *ts, struct timespec *tdelta, struct sload *l)
{
	uint64_t deltaus;
	int i;

	deltaus = tdelta->tv_sec*1000000+tdelta->tv_nsec/1000;

	for (i=0; i<instances; i++) {
		// calculate our load
		calc_load(ts, deltaus, &(load[i]));

		// adjust it for our needs
		tune_load(ts, &(load[i]));
	}
}

int main(int argc, char *argv[])
{
	int status;
	struct sched_param threadsched;
	pthread_t t;
	struct timespec period;
	char strname[16];
	size_t i;

	if (argc >= 2) {
		requested_load = atof(argv[1]);

		if ((requested_load > 100.0) || (requested_load < 0.0))
			error();
	}

	if (argc >= 3) {
		instances = atoi(argv[2]);
		if ((instances > MAX_INSTANCES) || (instances < 1)) {
			printf("supporting instances between 1 and %d\n", MAX_INSTANCES);
			error();
		}
		if (instances*(int)requested_load > 100) {
			printf("Sum of all instances generates larger than 100% load which is unsupported\n");
			error();
		}
	} else {
		instances = 1;
	}

	ticks_per_second = sysconf(_SC_CLK_TCK);
	us_per_tick = 1000000/ticks_per_second;

	// set our period time to 1 tick period
	dtns = 1*us_per_tick*1000;

	for (i=0; i<instances; i++) {
		status = pthread_create(&t, NULL, dummy_load, &(load[i]));

		sprintf(strname, "dummy_load-%d", i);
		status = pthread_setname_np(t, strname);

		threadsched.sched_priority = 37;
		status = sched_setscheduler(0, SCHED_FIFO, &threadsched);
		if (status) {
			printf("%s: sched_setscheduler %d\n", __FUNCTION__, status);
			pthread_exit(&status);
		}
	}

	pid = getpid();

	period.tv_sec = 0;
	period.tv_nsec = (uint64_t)dtns;
	printf("period for pid regulator is set to %ld s %ld ns\n",
		period.tv_sec, period.tv_nsec);

	periodic_loop(&period, worker, NULL);
}
