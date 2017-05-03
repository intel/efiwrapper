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

#include <efi.h>
#include <efiapi.h>
#include <interface.h>
#include <ewlog.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <string.h>
#include <errno.h>

#include "worker.h"
#include "tcp4.h"

static EFI_SYSTEM_TABLE *saved_st;

static EFI_GUID tcp4_guid = EFI_TCP4_PROTOCOL;
static worker_t cleaner;

typedef struct tcp4 {
	EFI_TCP4 prot;
	int fd;
	worker_t accepter;
	EFI_HANDLE handle;
	EFI_TCP4_CLOSE_TOKEN *close_token;
	worker_t reader;
	worker_t writer;
	UINT16 port;
	EFI_TCP4_CONNECTION_STATE state;
	EFI_TCP4_CONFIG_DATA config;
	EFI_IP4_MODE_DATA mode;
} tcp4_t;

static EFIAPI EFI_STATUS
tcp4_get_mode_data(EFI_TCP4 *This,
		   EFI_TCP4_CONNECTION_STATE *Tcp4State,
		   EFI_TCP4_CONFIG_DATA *Tcp4ConfigData,
		   EFI_IP4_MODE_DATA *Ip4ModeData,
		   EFI_MANAGED_NETWORK_CONFIG_DATA *MnpConfigData,
		   EFI_SIMPLE_NETWORK_MODE *SnpModeData)
{
	tcp4_t *tcp = (tcp4_t *)This;

	if (!This)
		return EFI_INVALID_PARAMETER;

	if (MnpConfigData || SnpModeData)
		return EFI_UNSUPPORTED;

	if (Tcp4State)
		memcpy(Tcp4State, &tcp->state, sizeof(tcp->state));

	if (Tcp4ConfigData)
		memcpy(Tcp4ConfigData, &tcp->config, sizeof(tcp->config));

	if (Ip4ModeData)
		memcpy(Ip4ModeData, &tcp->mode, sizeof(tcp->mode));

	return EFI_SUCCESS;
}

static EFI_STATUS do_read(tcp4_t *tcp, EFI_TCP4_IO_TOKEN *token)
{
	EFI_TCP4_RECEIVE_DATA *data;
	ssize_t count;

	data = token->Packet.RxData;
	count = read(tcp->fd, data->FragmentTable[0].FragmentBuffer,
		    data->FragmentTable[0].FragmentLength);
	if (count == -1) {
		ewerr("reader: failed to read, %s", strerror(errno));
		return EFI_DEVICE_ERROR;
	} else if (count == 0)
		token->CompletionToken.Status = EFI_CONNECTION_FIN;
	else
		token->CompletionToken.Status = EFI_SUCCESS;

	data->FragmentTable[0].FragmentLength = count;

	return EFI_SUCCESS;
}

static EFI_STATUS do_write(tcp4_t *tcp, EFI_TCP4_IO_TOKEN *token)
{
	EFI_TCP4_TRANSMIT_DATA *data;
	ssize_t count;

	data = token->Packet.TxData;
	count = write(tcp->fd, data->FragmentTable[0].FragmentBuffer,
		      data->FragmentTable[0].FragmentLength);
	if (count == -1) {
		ewerr("writer: failed to write, %s", strerror(errno));
		return EFI_DEVICE_ERROR;
	}
	token->CompletionToken.Status = EFI_SUCCESS;

	data->FragmentTable[0].FragmentLength = count;

	return EFI_SUCCESS;
}

typedef EFI_STATUS (*doer_t)(tcp4_t *, EFI_TCP4_IO_TOKEN *);

static EFI_STATUS read_write_routine(worker_t worker, doer_t fun)
{
	EFI_STATUS ret;
	tcp4_t *tcp;
	EFI_TCP4_IO_TOKEN *token;

	ret = worker_get_priv(worker, (void **)&tcp);
	if (EFI_ERROR(ret))
		return ret;

	for (;;) {
		ret = worker_get_job(worker, (void **)&token);
		if (ret == EFI_NOT_FOUND)
			return EFI_SUCCESS;
		if (EFI_ERROR(ret)) {
			ewerr("reader/writer: failed to get the next job");
			return ret;
		}

		ret = fun(tcp, token);
		if (EFI_ERROR(ret))
			return ret;

		ret = uefi_call_wrapper(saved_st->BootServices->SignalEvent, 1,
					token->CompletionToken.Event);
		if (EFI_ERROR(ret)) {
			ewerr("reader/writer: failed to signal event");
			return ret;
		}
	}

	return EFI_SUCCESS;
}

static EFI_STATUS read_routine(worker_t worker)
{
	return read_write_routine(worker, do_read);
}

static EFI_STATUS write_routine(worker_t worker)
{
	return read_write_routine(worker, do_write);
}

static EFI_STATUS accept_routine(worker_t worker)
{
	EFI_STATUS ret;
	struct sockaddr_in addr;
	socklen_t len;
	int fd;
	tcp4_t *tcp, *new_tcp;
	EFI_TCP4_LISTEN_TOKEN *token;

	ret = worker_get_priv(worker, (void **)&tcp);
	if (EFI_ERROR(ret))
		return ret;

	for (;;) {
		ret = worker_get_job(worker, (void **)&token);
		if (ret == EFI_NOT_FOUND)
			return EFI_SUCCESS;
		if (EFI_ERROR(ret)) {
			ewerr("accepter: failed to get next job");
			return ret;
		}

		len = sizeof(addr);
		fd = accept(tcp->fd, (struct sockaddr *)&addr, &len);
		if (fd == -1) {
			ewerr("accepter: accept failed, %s", strerror(errno));
			return EFI_DEVICE_ERROR;
		}

		token->NewChildHandle = NULL;
		ret = interface_init(saved_st, &tcp4_guid, &token->NewChildHandle,
				     tcp, sizeof(*tcp), (void **)&new_tcp);
		if (EFI_ERROR(ret)) {
			close(fd);
			ewerr("accepter: failed to create new connection interface");
			return ret;
		}

		new_tcp->fd = fd;
		new_tcp->state = Tcp4StateEstablished;
		new_tcp->handle = token->NewChildHandle;

		ret = worker_start(read_routine, new_tcp, &new_tcp->reader);
		if (EFI_ERROR(ret)) {
			ewerr("accepter: failed to start the reader");
			return ret;
		}

		ret = worker_start(write_routine, new_tcp, &new_tcp->writer);
		if (EFI_ERROR(ret)) {
			ewerr("accepter: failed to start the writer");
			return ret;
		}

		token->CompletionToken.Status = EFI_SUCCESS;
		ret = uefi_call_wrapper(saved_st->BootServices->SignalEvent, 1,
					token->CompletionToken.Event);
		if (EFI_ERROR(ret)) {
			ewerr("accepter: failed to signal the event");
			return ret;
		}
	}

	return EFI_SUCCESS;
}

static EFI_STATUS
clean_tcp(tcp4_t *tcp)
{
	EFI_STATUS ret;

	if (tcp->fd != -1) {
		close(tcp->fd);
		tcp->fd = -1;
	}

	switch (tcp->state) {
	case Tcp4StateListen:
		ret = worker_stop(tcp->accepter);
		if (EFI_ERROR(ret))
			return ret;

		return worker_free(tcp->accepter);

	case Tcp4StateEstablished:
		ret = interface_free(saved_st, &tcp4_guid, tcp->handle);
		if (EFI_ERROR(ret))
			ewerr("Failed to uninstall the connection interface");
		return ret;

	default:
		return EFI_INVALID_PARAMETER;
	}
}

static EFIAPI EFI_STATUS
tcp4_configure(EFI_TCP4 *This,
	       EFI_TCP4_CONFIG_DATA *TcpConfigData)
{
	EFI_STATUS ret = EFI_DEVICE_ERROR;
	tcp4_t *tcp = (tcp4_t *)This;
	EFI_IPv4_ADDRESS any = { { 0, 0, 0, 0 } };
	struct sockaddr_in addr;
	int pret, reuseval;

	if (!This)
		return EFI_INVALID_PARAMETER;

	if (!TcpConfigData)
		return clean_tcp(tcp);

	if (!TcpConfigData->AccessPoint.UseDefaultAddress ||
	    memcmp(&any, &TcpConfigData->AccessPoint.StationAddress, sizeof(any)) ||
	    memcmp(&any, &TcpConfigData->AccessPoint.RemoteAddress, sizeof(any)) ||
	    TcpConfigData->AccessPoint.RemotePort != 0)
		return EFI_UNSUPPORTED;

	tcp->port = TcpConfigData->AccessPoint.StationPort;
	tcp->fd = socket(AF_INET, SOCK_STREAM, 0);
	if (tcp->fd == -1) {
		ewerr("Failed to open a TCP socket, %s", strerror(errno));
		return EFI_DEVICE_ERROR;
	}

	reuseval = 1;
	pret = setsockopt(tcp->fd, SOL_SOCKET, SO_REUSEADDR, &reuseval,
			 sizeof(reuseval));
	if (pret == -1) {
		ewerr("Failed to set REUSEADDR flag, %s", strerror(errno));
		goto err;
	}

	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(tcp->port);
	addr.sin_addr.s_addr = INADDR_ANY;

	pret = bind(tcp->fd, (struct sockaddr *)&addr, sizeof(addr));
	if (pret == -1) {
		ewerr("Failed to bind the TCP socket, %s", strerror(errno));
		goto err;
	}

	pret = listen(tcp->fd, 5);
	if (pret == -1) {
		ewerr("Failed to listen on the TCP socket, %s",
		      strerror(errno));
		goto err;
	}

	tcp->state = Tcp4StateListen;
	memcpy(&tcp->config, TcpConfigData, sizeof(tcp->config));

	tcp->mode.ConfigData.StationAddress.Addr[0] = 127;
	tcp->mode.ConfigData.StationAddress.Addr[1] = 0;
	tcp->mode.ConfigData.StationAddress.Addr[2] = 0;
	tcp->mode.ConfigData.StationAddress.Addr[3] = 1;
	tcp->mode.IsConfigured = TRUE;

	ret = worker_start(accept_routine, tcp, &tcp->accepter);
	if (EFI_ERROR(ret)) {
		ewerr("Failed to start the accepter");
		goto err;
	}

	return EFI_SUCCESS;

err:
	close(tcp->fd);
	tcp->fd = -1;
	return ret;
}

static EFIAPI EFI_STATUS
tcp4_routes(__attribute__((__unused__)) EFI_TCP4 *This,
	    __attribute__((__unused__)) BOOLEAN DeleteRoute,
	    __attribute__((__unused__)) EFI_IPv4_ADDRESS *SubnetAddress,
	    __attribute__((__unused__)) EFI_IPv4_ADDRESS *SubnetMask,
	    __attribute__((__unused__)) EFI_IPv4_ADDRESS *GatewayAddress)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
tcp4_connect(__attribute__((__unused__)) EFI_TCP4 *This,
	     __attribute__((__unused__)) EFI_TCP4_CONNECTION_TOKEN *ConnectionToken)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
tcp4_accept(EFI_TCP4 *This,
	    EFI_TCP4_LISTEN_TOKEN *ListenToken)
{
	tcp4_t *tcp = (tcp4_t *)This;

	if (!This || !ListenToken)
		return EFI_INVALID_PARAMETER;

	if (tcp->state != Tcp4StateListen || !tcp->accepter)
		return EFI_NOT_STARTED;

	return worker_put_job(tcp->accepter, ListenToken);
}

static EFIAPI EFI_STATUS
tcp4_transmit(EFI_TCP4 *This,
	      EFI_TCP4_IO_TOKEN *Token)
{
	tcp4_t *tcp = (tcp4_t *)This;

	if (!This || !Token)
		return EFI_INVALID_PARAMETER;

	if (tcp->state != Tcp4StateEstablished || !tcp->writer)
		return EFI_NOT_STARTED;

	return worker_put_job(tcp->writer, Token);
}

static EFIAPI EFI_STATUS
tcp4_receive(EFI_TCP4 *This,
	     EFI_TCP4_IO_TOKEN *Token)
{
	tcp4_t *tcp = (tcp4_t *)This;

	if (!This || !Token)
		return EFI_INVALID_PARAMETER;

	if (tcp->state != Tcp4StateEstablished || !tcp->reader)
		return EFI_NOT_STARTED;

	return worker_put_job(tcp->reader, Token);
}

static EFIAPI EFI_STATUS
tcp4_close(EFI_TCP4 *This,
	   EFI_TCP4_CLOSE_TOKEN *CloseToken)
{
	EFI_STATUS ret;
	tcp4_t *tcp = (tcp4_t *)This;

	if (!This || !CloseToken)
		return EFI_INVALID_PARAMETER;

	if (tcp->state != Tcp4StateEstablished || !tcp->reader || !tcp->writer)
		return EFI_NOT_STARTED;

	close(tcp->fd);

	ret = worker_stop(tcp->reader);
	if (EFI_ERROR(ret)) {
		ewerr("Failed to stop the reader");
		return ret;
	}

	ret = worker_stop(tcp->writer);
	if (EFI_ERROR(ret)) {
		ewerr("Failed to stop the writer");
		return ret;
	}

	tcp->close_token = CloseToken;
	ret = worker_put_job(cleaner, tcp);
	if (EFI_ERROR(ret)) {
		ewerr("Failed to push the TCP connection to the cleaner");
		return ret;
	}

	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
tcp4_cancel(__attribute__((__unused__)) EFI_TCP4 *This,
	    __attribute__((__unused__)) EFI_TCP4_COMPLETION_TOKEN *Token)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
tcp4_poll(__attribute__((__unused__)) EFI_TCP4 *This)
{
	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
create_child(__attribute__((__unused__)) EFI_SERVICE_BINDING *This,
	     EFI_HANDLE *ChildHandle)
{
	static tcp4_t tcp4_default = {
		.prot = {
			.GetModeData = tcp4_get_mode_data,
			.Configure = tcp4_configure,
			.Routes = tcp4_routes,
			.Connect = tcp4_connect,
			.Accept = tcp4_accept,
			.Transmit = tcp4_transmit,
			.Receive = tcp4_receive,
			.Close = tcp4_close,
			.Cancel = tcp4_cancel,
			.Poll = tcp4_poll
		},
		.fd = -1,
		.state = Tcp4StateClosed,
	};
	EFI_TCP4 *tcp4;

	if (!ChildHandle)
		return EFI_INVALID_PARAMETER;

	*ChildHandle = NULL;
	return interface_init(saved_st, &tcp4_guid, ChildHandle,
			      &tcp4_default, sizeof(tcp4_default),
			      (void **)&tcp4);
}

static EFIAPI EFI_STATUS
destroy_child(__attribute__((__unused__)) EFI_SERVICE_BINDING *This,
	      EFI_HANDLE ChildHandle)
{
	return interface_free(saved_st, &tcp4_guid, ChildHandle);
}

static EFI_GUID srv_guid = EFI_TCP4_SERVICE_BINDING_PROTOCOL;
static EFI_HANDLE srv_handle;

static EFI_STATUS cleaner_routine(worker_t worker)
{
	EFI_STATUS ret;
	tcp4_t *conn;

	for (;;) {
		ret = worker_get_job(worker, (void **)&conn);
		if (ret == EFI_NOT_FOUND)
			return EFI_SUCCESS;
		if (EFI_ERROR(ret)) {
			ewerr("cleaner: failed to get the next job");
			return ret;
		}

		ret = worker_free(conn->reader);
		if (EFI_ERROR(ret))
			ewerr("cleaner: failed free to free the reader");

		ret = worker_free(conn->writer);
		if (EFI_ERROR(ret))
			ewerr("cleaner: failed free to free the writer");

		ret = uefi_call_wrapper(saved_st->BootServices->SignalEvent, 1,
					conn->close_token->CompletionToken.Event);
		if (EFI_ERROR(ret)) {
			ewerr("cleaner: failed to signal the closed connection");
			return ret;
		}
	}

	return EFI_SUCCESS;
}

static EFI_STATUS tcp4_init(EFI_SYSTEM_TABLE *st)
{
	static EFI_SERVICE_BINDING srv_default = {
		.CreateChild = create_child,
		.DestroyChild = destroy_child
	};
	EFI_SERVICE_BINDING *srv;
	EFI_STATUS ret;

	if (!st)
		return EFI_INVALID_PARAMETER;
	saved_st = st;

	ret = interface_init(st, &srv_guid, &srv_handle,
			     &srv_default, sizeof(srv_default),
			     (void **)&srv);
	if (EFI_ERROR(ret))
		return ret;

	ret = worker_start(cleaner_routine, NULL, &cleaner);
	if (EFI_ERROR(ret)) {
		ewerr("Failed to start the cleaner");
		srv_handle = saved_st = NULL;
		interface_free(st, &srv_guid, srv_handle);
	}

	return ret;
}

static EFI_STATUS tcp4_exit(EFI_SYSTEM_TABLE *st)
{
	EFI_STATUS ret;

	if (!st)
		return EFI_INVALID_PARAMETER;

	ret = worker_stop(cleaner);
	if (EFI_ERROR(ret)) {
		ewerr("Failed to stop the cleaner worker");
		return ret;
	}

	ret = worker_free(cleaner);
	if (EFI_ERROR(ret)) {
		ewerr("Failed to free the cleaner worker");
		return ret;
	}

	ret = interface_free(st, &srv_guid, srv_handle);
	srv_handle = saved_st = NULL;
	if (EFI_ERROR(ret))
		return ret;

	return EFI_SUCCESS;
}

ewdrv_t tcp4_drv = {
	.name = "tcp4",
	.description = "TCP/IP protocol",
	.init = tcp4_init,
	.exit = tcp4_exit
};
