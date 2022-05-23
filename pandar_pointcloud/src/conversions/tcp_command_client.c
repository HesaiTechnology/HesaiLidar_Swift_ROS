/******************************************************************************
 * Copyright 2019 The Hesai Technology Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <pthread.h>
#include <setjmp.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <syslog.h>
#include <unistd.h>
#include <linux/sockios.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include "util.h"
#include "tcp_command_client.h"

typedef struct TcpCommandClient_s {
  pthread_mutex_t lock;
  pthread_t tid;

  int exit;

  char ip[256];
  unsigned short port;

  int fd;
} TcpCommandClient;

char *certFile;
char *privateKeyFile;
char *caFile;
CERTIFY_MODE sslFlag = 0;

#ifdef DEBUG
static void print_mem(char* mem, int len) {
  int i = 0;
  for (int i = 0; i < len; ++i) {
    printf("%02x ", mem[i]);
  }
  printf("\n");
}
#else
static void print_mem(char* mem, int len) {}
#endif

static int tcpCommandHeaderParser(unsigned char* buffer, int len,
                                  TcpCommandHeader* header) {
  int index = 0;
  header->cmd = buffer[index++];
  header->ret_code = buffer[index++];
  header->len =
      ((buffer[index] & 0xff) << 24) | ((buffer[index + 1] & 0xff) << 16) |
      ((buffer[index + 2] & 0xff) << 8) | ((buffer[index + 3] & 0xff) << 0);
  return 0;
}

static int tcpCommandReadCommand(int connfd, TC_Command* cmd) {
  int ret = 0;
  if (!cmd) {
    return -1;
  }
  memset(cmd, 0, sizeof(TC_Command));
  unsigned char buffer[1500];
  ret = sys_readn(connfd, buffer, 2);
  if (ret <= 0 || buffer[0] != 0x47 || buffer[1] != 0x74) {
    printf("Server Read failed\n");
    return -1;
  }

  ret = sys_readn(connfd, buffer + 2, 6);
  if (ret != 6) {
    printf("Server Read failed\n");
    return -1;
  }

  print_mem(buffer, 8);

  tcpCommandHeaderParser(buffer + 2, 6, &cmd->header);

  if (cmd->header.len > 0) {
    cmd->data = malloc(cmd->header.len);
    if (!cmd->data) {
      printf("malloc data error\n");
      return -1;
    }
  }

  ret = sys_readn(connfd, cmd->data, cmd->header.len);
  if (ret != cmd->header.len) {
    free(cmd->data);
    printf("Server Read failed\n");
    return -1;
  }

  // cmd->ret_size = cmd->header.len;

  print_mem(cmd->data, cmd->header.len);

  return 0;
}

static int tcpCommandReadCommandBySSL(SSL *ssl, TC_Command* cmd) {
	int ret = 0;
	if(!cmd) {
		return -1;
	}
	memset(cmd , 0 , sizeof(TC_Command));
	unsigned char buffer [1500];
  printf("reading data....\n");
	ret = sys_readn_by_ssl(ssl, buffer , 2);
	if(ret <= 0 || buffer[0] != 0x47 || buffer [1] != 0x74) {
    printf("ret: %d, buffer[0]: %x,buffer[1]: %x\n",ret, buffer[0], buffer[1]);
		printf("Server Read failed by ssl!!!\n");
		return -1;
	}

	ret = sys_readn_by_ssl(ssl, buffer + 2 , 6);
	if(ret != 6) {
		printf("Server Read failed by ssl\n");
		return -1;
	}
	
	// printf("read response header size = 8:\n");
	print_mem(buffer , 8);
	tcpCommandHeaderParser(buffer + 2 , 6 , &cmd->header);

	if(cmd->header.len > 0) {
		cmd->data = malloc(cmd->header.len + 1);
		if(!cmd->data) {
			printf("malloc data error\n");
			return -1;
		}
		memset(cmd->data, 0, cmd->header.len + 1);
	}

	ret = sys_readn_by_ssl(ssl, cmd->data , cmd->header.len);
	if(ret != cmd->header.len) {
		free(cmd->data);
		printf("Server Read failed by ssl\n");
		return -1;
	}
	printf("read response data size = %d:\n", cmd->header.len);
	print_mem(cmd->data , cmd->header.len);
	cmd->ret_data = cmd->data;
	cmd->ret_size = cmd->header.len;
	return 0;
}

void BuildCmd(TC_Command command, PTC_COMMAND cmd, unsigned char* data){;
  // memset(&cmd, 0, sizeof(TC_Command));
  command.header.cmd = cmd;
  command.header.len = sizeof(data);
  command.data = data;
  return;
}

static int TcpCommand_buildHeader(char* buffer, TC_Command* cmd) {
  if (!buffer) {
    return -1;
  }
  int index = 0;
  buffer[index++] = 0x47;
  buffer[index++] = 0x74;
  buffer[index++] = cmd->header.cmd;
  buffer[index++] = cmd->header.ret_code;  // color or mono
  buffer[index++] = (cmd->header.len >> 24) & 0xff;
  buffer[index++] = (cmd->header.len >> 16) & 0xff;
  buffer[index++] = (cmd->header.len >> 8) & 0xff;
  buffer[index++] = (cmd->header.len >> 0) & 0xff;

  return index;
}

static PTC_ErrCode tcpCommandClientSendCmdWithoutSecurity(TcpCommandClient* client,
                                            TC_Command* cmd) {
  if (!client && !cmd) {
    printf("Bad Parameter\n");
    return PTC_ERROR_BAD_PARAMETER;
  }

  if (cmd->header.len != 0 && cmd->data == NULL) {
    printf("Bad Parameter : payload is null\n");
    return PTC_ERROR_BAD_PARAMETER;
  }
  pthread_mutex_lock(&client->lock);

  int fd = tcp_open(client->ip, client->port);
  if (fd < 0) {
    pthread_mutex_unlock(&client->lock);
     printf("connect server failed\n");
    return PTC_ERROR_CONNECT_SERVER_FAILED;
  }
  unsigned char buffer[128];
  int size = TcpCommand_buildHeader(buffer, cmd);

  print_mem(buffer, size);
  int ret = write(fd, buffer, size);
  if (ret != size) {
    close(fd);
    pthread_mutex_unlock(&client->lock);
    printf("Write header error\n");
    return PTC_ERROR_TRANSFER_FAILED;
  }
  if (cmd->header.len > 0 && cmd->data) {
    print_mem(cmd->data, cmd->header.len);
    ret = write(fd, cmd->data, cmd->header.len);
    if (ret != cmd->header.len) {
      printf("Write Payload error\n");
      close(fd);
      pthread_mutex_unlock(&client->lock);
      return PTC_ERROR_TRANSFER_FAILED;
    }
  }

  TC_Command feedBack;
  ret = tcpCommandReadCommand(fd, &feedBack);
  if (ret != 0) {
    printf("Receive feed back failed!!!\n");
    close(fd);
    pthread_mutex_unlock(&client->lock);
    return PTC_ERROR_TRANSFER_FAILED;
  }
  // printf("feed back : %d %d %d \n", feedBack.header.len ,
  // cmd->header.ret_code,
  //        cmd->header.cmd);

  cmd->ret_data = feedBack.data;
  cmd->ret_size = feedBack.header.len;
  cmd->header.ret_code = feedBack.header.ret_code;

  close(fd);
  pthread_mutex_unlock(&client->lock);
  return PTC_ERROR_NO_ERROR;
}

static PTC_ErrCode tcpCommandClientSendCmdWithSecurity(TcpCommandClient *client , TC_Command *cmd) {
	if(!client || !cmd) {
		printf("Bad Parameter\n");
		return PTC_ERROR_BAD_PARAMETER;
	}

	if(cmd->header.len != 0 && cmd->data == NULL) {
		printf("Bad Parameter : payload is null\n");
		return PTC_ERROR_BAD_PARAMETER;
	}
	pthread_mutex_lock(&client->lock);
	int err_code = PTC_ERROR_NO_ERROR;

	SSL_CTX* ctx = initial_client_ssl(certFile, privateKeyFile, caFile);
	if(ctx == NULL) {
    printf("%s:%d, create SSL_CTX failed\n", __func__, __LINE__);
		// ERR_print_errors_fp(stderr);
		return PTC_ERROR_CONNECT_SERVER_FAILED;
	}

	int fd = tcp_open(client->ip , client->port);
	if(fd < 0) {
		printf("Connect to Server Failed!~!~\n");
		err_code = PTC_ERROR_CONNECT_SERVER_FAILED;
		goto end;
	}

	SSL *ssl = SSL_new(ctx);
	if (ssl == NULL) {
		printf("%s:%d, create ssl failed\n", __func__, __LINE__);
		err_code = PTC_ERROR_CONNECT_SERVER_FAILED;
		goto end;
	}

	SSL_set_fd(ssl, fd);
	if(SSL_connect(ssl) == 0) {
		printf("%s:%d, connect ssl failed\n", __func__, __LINE__);
		// ERR_print_errors_fp(stderr);
		err_code = PTC_ERROR_CONNECT_SERVER_FAILED;
		goto end;
	}

	if(SSL_get_verify_result(ssl) != X509_V_OK) {
		printf("%s:%d, verify ssl failed\n", __func__, __LINE__);
		// ERR_print_errors_fp(stderr);
		err_code = PTC_ERROR_CONNECT_SERVER_FAILED;
		goto end;
	}

	unsigned char buffer[128];
	int size = TcpCommand_buildHeader(buffer , cmd);
	// printf("cmd header to tx, size = %d: \n",size);
	print_mem(buffer , size);
	int ret = SSL_write(ssl , buffer , size);
	if(ret != size) {
		printf("Write header error, ret=%d, size=%d\n", ret, size);
		err_code = PTC_ERROR_TRANSFER_FAILED;
		goto end;
	}

	if(cmd->header.len > 0 && cmd->data) {		
		printf(" cmd data to tx size = %d: \n", cmd->header.len);
		print_mem(cmd->data , cmd->header.len);
		ret = SSL_write(ssl, cmd->data , cmd->header.len);
		if(ret != cmd->header.len) {
			printf("Write Payload error\n");
			err_code = PTC_ERROR_TRANSFER_FAILED;
			goto end;
		}
	}

	TC_Command feedBack;
	ret = tcpCommandReadCommandBySSL(ssl, &feedBack);
	if(ret != 0) {
		printf("Receive feed back failed!!!\n");
		err_code = PTC_ERROR_TRANSFER_FAILED;
		goto end;
	}
	// printf("feed back : %d %d %d \n", cmd->ret_size , cmd->header.ret_code , cmd->header.cmd);

	cmd->ret_data = feedBack.ret_data;
	cmd->ret_size = feedBack.ret_size;
	cmd->header.ret_code = feedBack.header.ret_code;
	printf("certify finished,close ssl and fd now \n");

	end:
	if (ssl != NULL) SSL_shutdown(ssl);
	if (fd > 0) close(fd);
	if (ctx != NULL) {
		SSL_CTX_free(ctx);
		pthread_mutex_unlock(&client->lock);
	}
	return err_code;
}

static PTC_ErrCode tcpCommandClient_SendCmd(TcpCommandClient *client, TC_Command *cmd) {
  if(CERTIFY_MODE_NONE == sslFlag) {
    printf("Get data without certification now...\n");
    return tcpCommandClientSendCmdWithoutSecurity(client, cmd);
  }
  if(CERTIFY_MODE_SINGLE == sslFlag) {
    printf("Get data with single certification now...\n");
    return tcpCommandClientSendCmdWithSecurity(client, cmd);
  }
  if(CERTIFY_MODE_DUAL == sslFlag) {
    printf("Get data with dual certification now...\n");
    return tcpCommandClientSendCmdWithSecurity(client, cmd);
  }
  if(CERTIFY_MODE_ERROR == sslFlag) {
    printf("No CA file found, please check CA file path!\n");
    return PTC_ERROR_BAD_PARAMETER;
  }
}

static PTC_ErrCode tcpCommandClient_RecieveCmd(TcpCommandClient *client, TC_Command *cmd, char** buffer,
                                          unsigned int* len) {
  PTC_ErrCode errorCode = tcpCommandClient_SendCmd(client, cmd);
  if (errorCode != PTC_ERROR_NO_ERROR) {
    printf("The client failed to send a command by TCP\n");
    return errorCode;
  }

  char* ret_str = (char*)malloc(cmd->ret_size + 1);
  memcpy(ret_str, cmd->ret_data, cmd->ret_size);
  ret_str[cmd->ret_size] = '\0';

  free(cmd->ret_data);

  *buffer = ret_str;
  *len = cmd->ret_size + 1;

  return cmd->header.ret_code;
}

void* TcpCommandClientNew(const char* ip, const unsigned short port) {
  if (!ip) {
    printf("Bad Parameter\n");
    return NULL;
  }

  TcpCommandClient* client =
      (TcpCommandClient*)malloc(sizeof(TcpCommandClient));
  if (!client) {
    printf("No Memory!!!\n");
    return NULL;
  }
  memset(client, 0, sizeof(TcpCommandClient));
  client->fd = -1;
  strncpy(client->ip, ip, strlen(ip));
  client->port = port;

  pthread_mutex_init(&client->lock, NULL);

  printf("TCP Command Client Init Success!!!\n");
  return (void*)client;
}

PTC_ErrCode TcpCommandSetCalibration(const void* handle, const char* buffer,
                                     unsigned int len) {
  if (!handle || !buffer || len <= 0) {
    printf("Bad Parameter!!!\n");
    return PTC_ERROR_BAD_PARAMETER;
  }
  TcpCommandClient* client = (TcpCommandClient*)handle;

  TC_Command cmd;
  memset(&cmd, 0, sizeof(TC_Command));
  cmd.header.cmd = PTC_COMMAND_SET_CALIBRATION;
  cmd.header.len = len;
  cmd.data = strdup(buffer);

  PTC_ErrCode errorCode = tcpCommandClient_SendCmd(client, &cmd);
  if (errorCode != PTC_ERROR_NO_ERROR) {
    free(cmd.data);
    return errorCode;
  }
  free(cmd.data);

  if (cmd.ret_data) {
    // useless data;
    free(cmd.ret_data);
  }

  return cmd.header.ret_code;
}

PTC_ErrCode TcpCommandGetCalibration(const void* handle, char** buffer,
                                     unsigned int* len) {
  if (!handle || !buffer || !len) {
    printf("Bad Parameter!!!\n");
    return PTC_ERROR_BAD_PARAMETER;
  }
  TcpCommandClient* client = (TcpCommandClient*)handle;

  TC_Command cmd;
  memset(&cmd, 0, sizeof(TC_Command));
  cmd.header.cmd = PTC_COMMAND_GET_CALIBRATION;
  cmd.header.len = 0;
  cmd.data = NULL;

  PTC_ErrCode errorCode = tcpCommandClient_SendCmd(client, &cmd);
  if (errorCode != PTC_ERROR_NO_ERROR) {
    return errorCode;
  }

  char* ret_str = (char*)malloc(cmd.ret_size + 1);
  memcpy(ret_str, cmd.ret_data, cmd.ret_size);
  ret_str[cmd.ret_size] = '\0';

  free(cmd.ret_data);

  *buffer = ret_str;
  *len = cmd.ret_size + 1;

  return cmd.header.ret_code;
}
PTC_ErrCode TcpCommandGetLidarCalibration(const void* handle, char** buffer,
                                          unsigned int* len) {
  return TcpCommandGet(handle, PTC_COMMAND_GET_LIDAR_CALIBRATION, buffer, len);
}

PTC_ErrCode TcpCommandGetLidarChannelConfig(const void* handle, char** buffer,
                                          unsigned int* len) {
  return TcpCommandGet(handle, PTC_COMMAND_GET_LIDAR_CHANNEL_CONFIG, buffer, len);
}

PTC_ErrCode TcpCommandGetLidarFiretime(const void* handle, char** buffer,
                                          unsigned int* len) {
  return TcpCommandGet(handle, PTC_COMMAND_GET_LIDAR_FIRETIMES, buffer, len);
}

PTC_ErrCode TcpCommandGetLidarLensHeatSwitch(const void* handle, unsigned char** buffer,
                                          unsigned int* len){
  PTC_ErrCode ret = TcpCommandGet(handle, PTC_COMMAND_GET_LIDAR_LENS_HEAT_SWITCH, buffer, len);
  return ret;                                         
}

PTC_ErrCode TcpCommandGetLidarStatus(const void* handle, unsigned char** buffer,
                                          unsigned int* len){
  PTC_ErrCode ret = TcpCommandGet(handle, PTC_COMMAND_GET_LIDAR_STATUS, buffer, len);
  return ret;                                         
}

PTC_ErrCode TcpCommandGetLidarConfigInfo(const void* handle, unsigned char** buffer,
                                          unsigned int* len){
  PTC_ErrCode ret = TcpCommandGet(handle, PTC_COMMAND_GET_LIDAR_CONFIG_INFO, buffer, len);
  return ret;                                         
}

PTC_ErrCode TcpCommandGet(const void* handle, PTC_COMMAND command, unsigned char** buffer, unsigned int* len){
  if (!handle || !buffer || !len) {
    printf("Bad Parameter!!!\n");
    return PTC_ERROR_BAD_PARAMETER;
  }
  TcpCommandClient* client = (TcpCommandClient*)handle;

  TC_Command cmd;
  memset(&cmd, 0, sizeof(TC_Command));
  cmd.header.cmd = command;
  cmd.header.len = 0;
  cmd.data = NULL;
  PTC_ErrCode errorCode = tcpCommandClient_RecieveCmd(client, &cmd, buffer, len);
  return errorCode;
}

PTC_ErrCode TcpCommandSetLidarStandbyMode(const void* handle) {
  uint8_t buff[] = {1};
  return TcpCommandSet(handle, PTC_COMMAND_SET_LIDAR_OPERATE_MODE, buff, sizeof(buff));
}

PTC_ErrCode TcpCommandSetLidarNormalMode(const void* handle) {
  uint8_t buff[] = {0};
  return TcpCommandSet(handle, PTC_COMMAND_SET_LIDAR_OPERATE_MODE, buff, sizeof(buff));
}

PTC_ErrCode TcpCommandSetLidarReturnMode(const void* handle, uint8_t mode) {
  uint8_t buff[] = {mode};
  return TcpCommandSet(handle, PTC_COMMAND_SET_LIDAR_RETURN_MODE, buff, sizeof(buff));
}

PTC_ErrCode TcpCommandSetLidarSpinRate(const void* handle, uint16_t spinRate) {
  uint8_t buff[2];
  buff[0] = (uint8_t)(spinRate >> 8);
  buff[1] = (uint8_t)(spinRate & 0xFF);
  return TcpCommandSet(handle, PTC_COMMAND_SET_LIDAR_SPIN_RATE, buff, sizeof(buff));
}

PTC_ErrCode TcpCommandSetLidarLensHeatSwitch(const void* handle, uint8_t heatSwitch) {
  uint8_t buff[] = {heatSwitch};
  return TcpCommandSet(handle, PTC_COMMAND_SET_LIDAR_LENS_HEAT_SWITCH, buff, sizeof(buff));
}

PTC_ErrCode TcpCommandSet(const void* handle, PTC_COMMAND cmd, unsigned char* data, uint32_t len){
  if (!handle) {
    printf("Bad Parameter!!!\n");
    return PTC_ERROR_BAD_PARAMETER;
  }
  TcpCommandClient* client = (TcpCommandClient*)handle;

  TC_Command command;
  memset(&command, 0, sizeof(TC_Command));
  command.header.cmd = cmd;
  command.header.len = len;
  command.data = data;
  return tcpCommandClient_SendCmd(client, &command);
}

PTC_ErrCode TcpCommandResetCalibration(const void* handle) {
  if (!handle) {
    printf("Bad Parameter!!!\n");
    return PTC_ERROR_BAD_PARAMETER;
  }
  TcpCommandClient* client = (TcpCommandClient*)handle;

  TC_Command cmd;
  memset(&cmd, 0, sizeof(TC_Command));
  cmd.header.cmd = PTC_COMMAND_RESET_CALIBRATION;
  cmd.header.len = 0;
  cmd.data = NULL;

  PTC_ErrCode errorCode = tcpCommandClient_SendCmd(client, &cmd);
  if (errorCode != PTC_ERROR_NO_ERROR) {
    return errorCode;
  }

  if (cmd.ret_data) {
    // useless data;
    free(cmd.ret_data);
  }

  return cmd.header.ret_code;
}

void TcpCommandClientDestroy(const void* handle) {}

void TcpCommandSetSsl(const char* cert, const char* private_key, const char* ca) {
  int len;
  sslFlag = CERTIFY_MODE_NONE;
  if(0 != strlen(ca)) {
    sslFlag = CERTIFY_MODE_SINGLE;
    len = strlen(ca);
    caFile = (char*)malloc(len*sizeof(char)+1);
    strcpy(caFile, ca);
  }
  if(0 != strlen(cert) && 0 != strlen(private_key)) {
    if(CERTIFY_MODE_SINGLE == sslFlag) {
      sslFlag = CERTIFY_MODE_DUAL;
      len = strlen(cert);
      certFile = (char*)malloc(len*sizeof(char)+1);
      strcpy(certFile, cert);
      len = strlen(private_key);
      privateKeyFile = (char*)malloc(len*sizeof(char)+1);
      strcpy(privateKeyFile, private_key);
    }
    else{
      sslFlag = CERTIFY_MODE_ERROR;
    }  
  }
}

SSL_CTX* initial_client_ssl(const char* cert, const char* private_key, const char* ca) {
	SSL_library_init();
	SSL_load_error_strings();
	OpenSSL_add_all_algorithms();
	SSL_CTX *ctx = SSL_CTX_new(SSLv23_client_method());

	if(ctx == NULL) {
		ERR_print_errors_fp(stderr);
		printf("%s:%d, create SSL_CTX failed\n", __func__, __LINE__);
		return NULL;
	}

	if (ca) {
    printf("ca path: %s\n",ca);
		if(	SSL_CTX_load_verify_locations(ctx, ca, NULL) == 0) {
			// ERR_print_errors_fp(stderr);
			printf("%s:%d, load ca failed,please check ca file path\n", __func__, __LINE__);
			return NULL;
		}
	}

	if (cert && private_key){
    printf("cert path: %s,\nprivate_key path: %s\n",cert, private_key);
		SSL_CTX_set_verify(ctx, SSL_VERIFY_PEER | SSL_VERIFY_FAIL_IF_NO_PEER_CERT, NULL);
		if(SSL_CTX_use_certificate_file(ctx, cert, SSL_FILETYPE_PEM) == 0) {
      printf("%s:%d, load cert file failed,please check cert file path\n", __func__, __LINE__);
			return NULL;
    }
    if(SSL_CTX_use_PrivateKey_file(ctx, private_key, SSL_FILETYPE_PEM) == 0) {
      printf("%s:%d, load private key file failed,please check private key file path\n", __func__, __LINE__);
			return NULL;
    }
    if(SSL_CTX_check_private_key(ctx) == 0) {
      printf("%s:%d, check private key failed\n", __func__, __LINE__);
			return NULL;
    }
	}
	return ctx;
}
