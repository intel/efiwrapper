/*
 * Copyright (c) 2016, Intel Corporation
 * All rights reserved.
 *
 * Author: Jérémy Compostella <jeremy.compostella@intel.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer
 *      in the documentation and/or other materials provided with the
 *      distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdlib.h>
#include <pthread.h>
#include <stdbool.h>
#include <ewlog.h>
#include <errno.h>
#include <string.h>

#include "fifo.h"
#include "worker.h"

struct worker {
	worker_routine routine;
	EFI_STATUS routine_ret;
	void *priv;
	bool running;
	pthread_t thread;
	fifo_t jobs;
	pthread_cond_t cond;
	pthread_mutex_t lock;
};

static inline EFI_STATUS lock(worker_t worker)
{
	int ret;

	ret = pthread_mutex_lock(&worker->lock);
	if (ret) {
		ewerr("Failed to lock worker");
		return EFI_DEVICE_ERROR;
	}

	return EFI_SUCCESS;
}

static inline EFI_STATUS unlock(worker_t worker)
{
	int ret;

	ret = pthread_mutex_unlock(&worker->lock);
	if (ret) {
		ewerr("Failed to unlock worker");
		return EFI_DEVICE_ERROR;
	}

	return EFI_SUCCESS;
}

static inline EFI_STATUS set_running(worker_t worker, bool running)
{
	EFI_STATUS ret;

	ret = lock(worker);
	if (EFI_ERROR(ret))
		return ret;

	worker->running = running;

	return unlock(worker);
}

static inline EFI_STATUS get_running(worker_t worker, bool *running)
{
	EFI_STATUS ret;

	ret = lock(worker);
	if (EFI_ERROR(ret))
		return ret;

	*running = worker->running;

	return unlock(worker);
}

static void *worker_run(void *arg)
{
	EFI_STATUS ret;
	worker_t worker;

	worker = (worker_t)arg;
	worker->routine_ret = worker->routine(worker);
	set_running(worker, false);

	return &worker->routine_ret;
}

EFI_STATUS worker_start(worker_routine routine, void *priv,
			worker_t *worker_p)
{
	int ret;
        worker_t worker;

	if (!routine || !worker_p)
		return EFI_INVALID_PARAMETER;

	worker = malloc(sizeof(*worker));
	if (!worker)
		return EFI_OUT_OF_RESOURCES;

	worker->jobs = fifo_new();
	if (!worker->jobs) {
		free(worker);
		return EFI_OUT_OF_RESOURCES;
	}

	worker->routine = routine;
	worker->priv = priv;

	ret = pthread_cond_init(&worker->cond, NULL);
	if (ret)
		goto err;

	ret = pthread_mutex_init(&worker->lock, NULL);
	if (ret)
		goto err;

	worker->running = true;
	ret = pthread_create(&worker->thread, NULL, worker_run, worker);
	if (ret)
		goto err;

	*worker_p = worker;
	return EFI_SUCCESS;

err:
	fifo_free(worker->jobs);
	free(worker);
	return EFI_DEVICE_ERROR;
}

EFI_STATUS worker_get_priv(worker_t worker, void **priv_p)
{
	if (!worker)
		return EFI_INVALID_PARAMETER;

	*priv_p = worker->priv;
	return EFI_SUCCESS;
}

static EFI_STATUS worker_sleep(worker_t worker)
{
	EFI_STATUS ret;
	int pret;

	ret = lock(worker);
	if (EFI_ERROR(ret))
		return ret;

        pret = pthread_cond_wait(&worker->cond, &worker->lock);
	if (pret)
		return EFI_DEVICE_ERROR;

	return unlock(worker);
}

EFI_STATUS worker_get_job(worker_t worker, void **job_p)
{
	EFI_STATUS ret;
	bool running;

	if (!worker || !job_p)
		return EFI_INVALID_PARAMETER;

	ret = get_running(worker, &running);
	if (EFI_ERROR(ret))
		return ret;
	if (!running)
		return EFI_NOT_FOUND;

	ret = fifo_get(worker->jobs, job_p);
	if (!EFI_ERROR(ret))
		return EFI_SUCCESS;
	if (ret != EFI_NOT_FOUND)
		return ret;

	ret = worker_sleep(worker);
	if (EFI_ERROR(ret))
		return ret;

	ret = get_running(worker, &running);
	if (EFI_ERROR(ret))
		return ret;
	if (!running)
		return EFI_NOT_FOUND;

	return fifo_get(worker->jobs, job_p);
}

static EFI_STATUS worker_wakeup(worker_t worker)
{
	EFI_STATUS ret;
	int pret;

	ret = lock(worker);
	if (EFI_ERROR(ret))
		return ret;

	pret = pthread_cond_signal(&worker->cond);
	if (pret)
		return EFI_DEVICE_ERROR;

	return unlock(worker);
}

EFI_STATUS worker_put_job(worker_t worker, void *job)
{
	EFI_STATUS ret;
	bool running;

	if (!worker || !job)
		return EFI_INVALID_PARAMETER;

	ret = get_running(worker, &running);
	if (EFI_ERROR(ret))
		return ret;
	if (!running)
		return EFI_NOT_FOUND;

	ret = fifo_put(worker->jobs, job);
	if (EFI_ERROR(ret))
		return ret;

	return worker_wakeup(worker);
}

EFI_STATUS worker_stop(worker_t worker)
{
	EFI_STATUS ret;

	if (!worker)
		return EFI_INVALID_PARAMETER;

	ret = set_running(worker, false);
	if (EFI_ERROR(ret)) {
		ewerr("Failed to stop the worker");
		return ret;
	}

	ret = worker_wakeup(worker);
	if (EFI_ERROR(ret))
		ewerr("Failed to wakeup the worker");

	return ret;
}

EFI_STATUS worker_free(worker_t worker)
{
	EFI_STATUS ret;
	void *retval;

	ret = pthread_join(worker->thread, &retval);
	if (ret) {
		ewerr("Failed to wait the thread termination, %s",
		      strerror(ret));
		return EFI_DEVICE_ERROR;
	}

	fifo_free(worker->jobs);
	free(worker);
	return EFI_SUCCESS;
}
