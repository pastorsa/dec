/*
 * dec_mutex.h
 *
 *  Created on: Jul 14, 2013
 *      Author: pastor
 */

#ifndef DEC_MUTEX_H_
#define DEC_MUTEX_H_

#include <stdio.h>
#include <pthread.h>
#include <sys/time.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef pthread_mutex_t dec_mutex;
typedef pthread_cond_t dec_cond;

typedef unsigned long long dec_time; // time in nanoseconds

static int dec_mutex_init(dec_mutex* mutex);
static int dec_mutex_destroy(dec_mutex* mutex);
static int dec_mutex_lock(dec_mutex* mutex);
static int dec_mutex_trylock(dec_mutex* mutex);
static int dec_mutex_unlock(dec_mutex* mutex);

static int dec_cond_init(dec_cond* cond);
static int dec_cond_destroy(dec_cond* cond);
static int dec_cond_signal(dec_cond* cond);
static int dec_cond_wait(dec_cond* cond, dec_mutex* mutex);
static int dec_cond_timedwait(dec_cond* cond, dec_mutex* mutex, dec_time timeout);
static int dec_cond_timedwait_relative(dec_cond* cond, dec_mutex* mutex, dec_time timeout);

#ifdef __cplusplus
}
#endif

/////////////////////////////////////////
// inline function definitions follow:

static inline int dec_mutex_init(dec_mutex* mutex)
{
  return pthread_mutex_init(mutex, NULL);
}

static inline int dec_mutex_destroy(dec_mutex* mutex)
{
  return pthread_mutex_destroy(mutex);
}

static inline int dec_mutex_lock(dec_mutex* mutex)
{
  return pthread_mutex_lock(mutex);
}

static inline int dec_mutex_trylock(dec_mutex* mutex)
{
  return pthread_mutex_trylock(mutex);
}

static inline int dec_mutex_unlock(dec_mutex* mutex)
{
  return pthread_mutex_unlock(mutex);
}

static inline int dec_cond_init(dec_cond* cond)
{
  return pthread_cond_init(cond, NULL);
}

static inline int dec_cond_destroy(dec_cond* cond)
{
  return pthread_cond_destroy(cond);
}

static inline int dec_cond_signal(dec_cond* cond)
{
  return pthread_cond_signal(cond);
}

static inline int dec_cond_wait(dec_cond* cond, dec_mutex* mutex)
{
  return pthread_cond_wait(cond, mutex);
}

static inline int dec_cond_timedwait(dec_cond* cond, dec_mutex* mutex, dec_time timeout)
{
  struct timespec ts;
  ts.tv_sec = (time_t) (timeout / 1000000000);
  ts.tv_nsec = (long) (timeout % 1000000000);
  return pthread_cond_timedwait(cond, mutex, &ts);
}

static inline int dec_cond_timedwait_relative(dec_cond* cond, dec_mutex* mutex, dec_time timeout)
{
  struct timeval t;
  gettimeofday(&t, NULL);

  struct timespec ts;
  ts.tv_sec = (time_t) (timeout / 1000000000);
  ts.tv_nsec = (long) (timeout % 1000000000);

  ts.tv_sec += t.tv_sec;
  ts.tv_nsec += t.tv_usec*1000;
  if (ts.tv_nsec >= 1000000000)
  {
    ts.tv_sec += 1;
    ts.tv_nsec -= 1000000000;
  }
  return pthread_cond_timedwait(cond, mutex, &ts);
}

#endif /* DEC_MUTEX_H_ */
