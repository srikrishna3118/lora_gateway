/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2013 Semtech-Cycleo

Description:
	Configure LoRa concentrator and record received packets in a log file

License: Revised BSD License, see LICENSE.TXT file include in the project
Maintainer: Sylvain Miermont
*/


/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

/* fix an issue between POSIX and C99 */
#if __STDC_VERSION__ >= 199901L
	#define _XOPEN_SOURCE 600
#else
	#define _XOPEN_SOURCE 500
#endif

#include <stdint.h>		/* C99 types */
#include <stdbool.h>	/* bool type */
#include <stdio.h>		/* printf fprintf sprintf fopen fputs */

#include <string.h>		/* memset */
#include <signal.h>		/* sigaction */
#include <time.h>		/* time clock_gettime strftime gmtime clock_nanosleep*/
#include <unistd.h>		/* getopt access */
#include <stdlib.h>		/* atoi */

#include "parson.h"
#include "loragw_hal.h"

/* header files to support socket programming */
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netdb.h>
#include <errno.h>
#include <arpa/inet.h>
#include <json/json.h>


/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a)	(sizeof(a) / sizeof((a)[0]))
#define MSG(args...)	fprintf(stderr,"loragw_pkt_logger: " args) /* message that is destined to the user */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES (GLOBAL) ------------------------------------------- */

/* signal handling variables */
struct sigaction sigact; /* SIGQUIT&SIGINT&SIGTERM signal handling */
static int exit_sig = 0; /* 1 -> application terminates cleanly (shut down hardware, close open files, etc) */
static int quit_sig = 0; /* 1 -> application terminates without shutting down the hardware */

/* configuration variables needed by the application  */
uint64_t lgwm = 0; /* LoRa gateway MAC address */
char lgwm_str[17];

/* clock and log file management */
time_t now_time;
time_t log_start_time;
FILE * log_file = NULL;
char log_file_name[64];

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DECLARATION ---------------------------------------- */

static void sig_handler(int sigio);

int parse_SX1301_configuration(const char * conf_file);

int parse_gateway_configuration(const char * conf_file);

void open_log(void);

void usage (void);

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DEFINITION ----------------------------------------- */

static void sig_handler(int sigio) {
	if (sigio == SIGQUIT) {
		quit_sig = 1;;
	} else if ((sigio == SIGINT) || (sigio == SIGTERM)) {
		exit_sig = 1;
	}
}

int parse_SX1301_configuration(const char * conf_file) {
	int i;
	const char conf_obj[] = "SX1301_conf";
	char param_name[32]; /* used to generate variable parameter names */
	struct lgw_conf_rxrf_s rfconf;
	struct lgw_conf_rxif_s ifconf;
	JSON_Value *root_val;
	JSON_Object *root = NULL;
	JSON_Object *conf = NULL;
	JSON_Value *val;
	uint32_t sf, bw;
	
	/* try to parse JSON */
	root_val = json_parse_file_with_comments(conf_file);
	root = json_value_get_object(root_val);
	if (root == NULL) {
		MSG("ERROR: %s id not a valid JSON file\n", conf_file);
		exit(EXIT_FAILURE);
	}
	conf = json_object_get_object(root, conf_obj);
	if (conf == NULL) {
		MSG("INFO: %s does not contain a JSON object named %s\n", conf_file, conf_obj);
		return -1;
	} else {
		MSG("INFO: %s does contain a JSON object named %s, parsing SX1301 parameters\n", conf_file, conf_obj);
	}
	
	/* set configuration for RF chains */
	for (i = 0; i < LGW_RF_CHAIN_NB; ++i) {
		memset(&rfconf, 0, sizeof(rfconf)); /* initialize configuration structure */
		sprintf(param_name, "radio_%i", i); /* compose parameter path inside JSON structure */
		val = json_object_get_value(conf, param_name); /* fetch value (if possible) */
		if (json_value_get_type(val) != JSONObject) {
			MSG("INFO: no configuration for radio %i\n", i);
			continue;
		}
		/* there is an object to configure that radio, let's parse it */
		sprintf(param_name, "radio_%i.enable", i);
		val = json_object_dotget_value(conf, param_name);
		if (json_value_get_type(val) == JSONBoolean) {
			rfconf.enable = (bool)json_value_get_boolean(val);
		} else {
			rfconf.enable = false;
		}
		if (rfconf.enable == false) { /* radio disabled, nothing else to parse */
			MSG("INFO: radio %i disabled\n", i);
		} else  { /* radio enabled, will parse the other parameters */
			sprintf(param_name, "radio_%i.freq", i);
			rfconf.freq_hz = (uint32_t)json_object_dotget_number(conf, param_name);
			MSG("INFO: radio %i enabled, center frequency %u\n", i, rfconf.freq_hz);
		}
		/* all parameters parsed, submitting configuration to the HAL */
		if (lgw_rxrf_setconf(i, rfconf) != LGW_HAL_SUCCESS) {
			MSG("WARNING: invalid configuration for radio %i\n", i);
		}
	}
	
	/* set configuration for LoRa multi-SF channels (bandwidth cannot be set) */
	for (i = 0; i < LGW_MULTI_NB; ++i) {
		memset(&ifconf, 0, sizeof(ifconf)); /* initialize configuration structure */
		sprintf(param_name, "chan_multiSF_%i", i); /* compose parameter path inside JSON structure */
		val = json_object_get_value(conf, param_name); /* fetch value (if possible) */
		if (json_value_get_type(val) != JSONObject) {
			MSG("INFO: no configuration for LoRa multi-SF channel %i\n", i);
			continue;
		}
		/* there is an object to configure that LoRa multi-SF channel, let's parse it */
		sprintf(param_name, "chan_multiSF_%i.enable", i);
		val = json_object_dotget_value(conf, param_name);
		if (json_value_get_type(val) == JSONBoolean) {
			ifconf.enable = (bool)json_value_get_boolean(val);
		} else {
			ifconf.enable = false;
		}
		if (ifconf.enable == false) { /* LoRa multi-SF channel disabled, nothing else to parse */
			MSG("INFO: LoRa multi-SF channel %i disabled\n", i);
		} else  { /* LoRa multi-SF channel enabled, will parse the other parameters */
			sprintf(param_name, "chan_multiSF_%i.radio", i);
			ifconf.rf_chain = (uint32_t)json_object_dotget_number(conf, param_name);
			sprintf(param_name, "chan_multiSF_%i.if", i);
			ifconf.freq_hz = (int32_t)json_object_dotget_number(conf, param_name);
			// TODO: handle individual SF enabling and disabling (spread_factor)
			MSG("INFO: LoRa multi-SF channel %i enabled, radio %i selected, IF %i Hz, 125 kHz bandwidth, SF 7 to 12\n", i, ifconf.rf_chain, ifconf.freq_hz);
		}
		/* all parameters parsed, submitting configuration to the HAL */
		if (lgw_rxif_setconf(i, ifconf) != LGW_HAL_SUCCESS) {
			MSG("WARNING: invalid configuration for LoRa multi-SF channel %i\n", i);
		}
	}
	
	/* set configuration for LoRa standard channel */
	memset(&ifconf, 0, sizeof(ifconf)); /* initialize configuration structure */
	val = json_object_get_value(conf, "chan_Lora_std"); /* fetch value (if possible) */
	if (json_value_get_type(val) != JSONObject) {
		MSG("INFO: no configuration for LoRa standard channel\n");
	} else {
		val = json_object_dotget_value(conf, "chan_Lora_std.enable");
		if (json_value_get_type(val) == JSONBoolean) {
			ifconf.enable = (bool)json_value_get_boolean(val);
		} else {
			ifconf.enable = false;
		}
		if (ifconf.enable == false) {
			MSG("INFO: LoRa standard channel %i disabled\n", i);
		} else  {
			ifconf.rf_chain = (uint32_t)json_object_dotget_number(conf, "chan_Lora_std.radio");
			ifconf.freq_hz = (int32_t)json_object_dotget_number(conf, "chan_Lora_std.if");
			bw = (uint32_t)json_object_dotget_number(conf, "chan_Lora_std.bandwidth");
			switch(bw) {
				case 500000: ifconf.bandwidth = BW_500KHZ; break;
				case 250000: ifconf.bandwidth = BW_250KHZ; break;
				case 125000: ifconf.bandwidth = BW_125KHZ; break;
				default: ifconf.bandwidth = BW_UNDEFINED;
			}
			sf = (uint32_t)json_object_dotget_number(conf, "chan_Lora_std.spread_factor");
			switch(sf) {
				case  7: ifconf.datarate = DR_LORA_SF7;  break;
				case  8: ifconf.datarate = DR_LORA_SF8;  break;
				case  9: ifconf.datarate = DR_LORA_SF9;  break;
				case 10: ifconf.datarate = DR_LORA_SF10; break;
				case 11: ifconf.datarate = DR_LORA_SF11; break;
				case 12: ifconf.datarate = DR_LORA_SF12; break;
				default: ifconf.datarate = DR_UNDEFINED;
			}
			MSG("INFO: LoRa standard channel enabled, radio %i selected, IF %i Hz, %u Hz bandwidth, SF %u\n", ifconf.rf_chain, ifconf.freq_hz, bw, sf);
		}
		if (lgw_rxif_setconf(8, ifconf) != LGW_HAL_SUCCESS) {
			MSG("WARNING: invalid configuration for LoRa standard channel\n");
		}
	}
	
	/* set configuration for FSK channel */
	memset(&ifconf, 0, sizeof(ifconf)); /* initialize configuration structure */
	val = json_object_get_value(conf, "chan_FSK"); /* fetch value (if possible) */
	if (json_value_get_type(val) != JSONObject) {
		MSG("INFO: no configuration for FSK channel\n");
	} else {
		val = json_object_dotget_value(conf, "chan_FSK.enable");
		if (json_value_get_type(val) == JSONBoolean) {
			ifconf.enable = (bool)json_value_get_boolean(val);
		} else {
			ifconf.enable = false;
		}
		if (ifconf.enable == false) {
			MSG("INFO: FSK channel %i disabled\n", i);
		} else  {
			ifconf.rf_chain = (uint32_t)json_object_dotget_number(conf, "chan_FSK.radio");
			ifconf.freq_hz = (int32_t)json_object_dotget_number(conf, "chan_FSK.if");
			bw = (uint32_t)json_object_dotget_number(conf, "chan_FSK.bandwidth");
			if      (bw <= 7800)   ifconf.bandwidth = BW_7K8HZ;
			else if (bw <= 15600)  ifconf.bandwidth = BW_15K6HZ;
			else if (bw <= 31200)  ifconf.bandwidth = BW_31K2HZ;
			else if (bw <= 62500)  ifconf.bandwidth = BW_62K5HZ;
			else if (bw <= 125000) ifconf.bandwidth = BW_125KHZ;
			else if (bw <= 250000) ifconf.bandwidth = BW_250KHZ;
			else if (bw <= 500000) ifconf.bandwidth = BW_500KHZ;
			else ifconf.bandwidth = BW_UNDEFINED;
			ifconf.datarate = (uint32_t)json_object_dotget_number(conf, "chan_FSK.datarate");
			MSG("INFO: FSK channel enabled, radio %i selected, IF %i Hz, %u Hz bandwidth, %u bps datarate\n", ifconf.rf_chain, ifconf.freq_hz, bw, ifconf.datarate);
		}
		if (lgw_rxif_setconf(9, ifconf) != LGW_HAL_SUCCESS) {
			MSG("WARNING: invalid configuration for FSK channel\n");
		}
	}
	json_value_free(root_val);
	return 0;
}

int parse_gateway_configuration(const char * conf_file) {
	const char conf_obj[] = "gateway_conf";
	JSON_Value *root_val;
	JSON_Object *root = NULL;
	JSON_Object *conf = NULL;
	unsigned long long ull = 0;
	
	/* try to parse JSON */
	root_val = json_parse_file_with_comments(conf_file);
	root = json_value_get_object(root_val);
	if (root == NULL) {
		MSG("ERROR: %s id not a valid JSON file\n", conf_file);
		exit(EXIT_FAILURE);
	}
	conf = json_object_get_object(root, conf_obj);
	if (conf == NULL) {
		MSG("INFO: %s does not contain a JSON object named %s\n", conf_file, conf_obj);
		return -1;
	} else {
		MSG("INFO: %s does contain a JSON object named %s, parsing gateway parameters\n", conf_file, conf_obj);
	}
	
	/* getting network parameters (only those necessary for the packet logger) */
	sscanf(json_object_dotget_string(conf, "gateway_ID"), "%llx", &ull);
	lgwm = ull;
	MSG("INFO: gateway MAC address is configured to %016llX\n", ull);
	
	json_value_free(root_val);
	return 0;
}

void open_log(void) {
	int i;
	char iso_date[20];
	
	strftime(iso_date,ARRAY_SIZE(iso_date),"%Y%m%dT%H%M%SZ",gmtime(&now_time)); /* format yyyymmddThhmmssZ */
	log_start_time = now_time; /* keep track of when the log was started, for log rotation */
	
	/*modified to save in same log file*/
	//sprintf(log_file_name, "pktlog_%s_%s.csv", lgwm_str, iso_date);
	sprintf(log_file_name, "pktlog_%s.csv", lgwm_str);
	log_file = fopen(log_file_name, "a"); /* create log file, append if file already exist */
	if (log_file == NULL) {
		MSG("ERROR: impossible to create log file %s\n", log_file_name);
		exit(EXIT_FAILURE);
	}
	
	i = fprintf(log_file, "\"gateway ID\",\"node MAC\",\"UTC timestamp\",\"us count\",\"frequency\",\"RF chain\",\"RX chain\",\"status\",\"size\",\"modulation\",\"bandwidth\",\"datarate\",\"coderate\",\"RSSI\",\"SNR\",\"payload\"\n");
	if (i < 0) {
		MSG("ERROR: impossible to write to log file %s\n", log_file_name);
		exit(EXIT_FAILURE);
	}
	
	MSG("INFO: Now writing to log file %s\n", log_file_name);
	return;
}

/* describe command line options */
void usage(void) {
	printf("*** Library version information ***\n%s\n\n", lgw_version_info());
	printf( "Available options:\n");
	printf( " -h print this help\n");
	printf( " -r <int> rotate log file every N seconds (-1 disable log rotation)\n");
}

int setupconf(void){

	/* configuration file related */
	const char global_conf_fname[] = "global_conf.json"; /* contain global (typ. network-wide) configuration */
	const char local_conf_fname[] = "local_conf.json"; /* contain node specific configuration, overwrite global parameters for parameters that are defined in both */
	const char debug_conf_fname[] = "debug_conf.json"; /* if present, all other configuration files are ignored */


	/* configuration files management */
	if (access(debug_conf_fname, R_OK) == 0) {
	/* if there is a debug conf, parse only the debug conf */
		MSG("INFO: found debug configuration file %s, other configuration files will be ignored\n", debug_conf_fname);
		parse_SX1301_configuration(debug_conf_fname);
		parse_gateway_configuration(debug_conf_fname);
	} else if (access(global_conf_fname, R_OK) == 0) {
	/* if there is a global conf, parse it and then try to parse local conf  */
		MSG("INFO: found global configuration file %s, trying to parse it\n", global_conf_fname);
		parse_SX1301_configuration(global_conf_fname);
		parse_gateway_configuration(global_conf_fname);
		if (access(local_conf_fname, R_OK) == 0) {
			MSG("INFO: found local configuration file %s, trying to parse it\n", local_conf_fname);
			parse_SX1301_configuration(local_conf_fname);
			parse_gateway_configuration(local_conf_fname);
		}
	} else if (access(local_conf_fname, R_OK) == 0) {
	/* if there is only a local conf, parse it and that's all */
		MSG("INFO: found local configuration file %s, trying to parse it\n", local_conf_fname);
		parse_SX1301_configuration(local_conf_fname);
		parse_gateway_configuration(local_conf_fname);
	} else {
		MSG("ERROR: failed to find any configuration file named %s, %s or %s\n", global_conf_fname, local_conf_fname, debug_conf_fname);
		return EXIT_FAILURE;
	}

	return 0;
}

/* -------------------------------------------------------------------------- */
/* --- MAIN LOOP FUNCTION -------------------------------------------------------- */

int execute()
{
	int i, j; /* loop and temporary variables */
	struct timespec sleep_time = {0, 3000000}; /* 3 ms */

	/* allocate memory for packet fetching and processing */
	struct lgw_pkt_rx_s rxpkt[16]; /* array containing up to 16 inbound packets metadata */
	struct lgw_pkt_rx_s *p; /* pointer on a RX packet */
	int nb_pkt;
	int corrupt_pkt_count=0;

	
	/* local timestamp variables until we get accurate GPS time */
	struct timespec fetch_time;
	char fetch_timestamp[30];
	struct tm * x;

	/*socket parameters*/
    int sockfd = 0;
    uint8_t recvBuff[256];
    memset(recvBuff, '0', sizeof(recvBuff));

    struct sockaddr_in serv_addr,smap_addr;

    /* Initialize sockaddr_in data structure */
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(1680); // port
    serv_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
	
	/* configure signal handling */
	sigemptyset(&sigact.sa_mask);
	sigact.sa_flags = 0;
	sigact.sa_handler = sig_handler;
	sigaction(SIGQUIT, &sigact, NULL);
	sigaction(SIGINT, &sigact, NULL);
	sigaction(SIGTERM, &sigact, NULL);

	
	/* starting the concentrator */
	i = lgw_start();
	if (i == LGW_HAL_SUCCESS) {
		MSG("INFO: concentrator started, packet can now be received\n");
	} else {
		MSG("ERROR: failed to start the concentrator\n");
		return EXIT_FAILURE;
	}
	
	/* main loop */
	while ((quit_sig != 1) && (exit_sig != 1)) {
		/* fetch packets */
		nb_pkt = lgw_receive(ARRAY_SIZE(rxpkt), rxpkt);
		if (nb_pkt == LGW_HAL_ERROR) {
			MSG("ERROR: failed packet fetch, exiting\n");
			return EXIT_FAILURE;
		} else if (nb_pkt == 0) {
			clock_nanosleep(CLOCK_MONOTONIC, 0, &sleep_time, NULL); /* wait a short time if no packets */
		}
		/* log packets */
		for (i=0; i < nb_pkt; ++i) {
			p = &rxpkt[i];
			if(p->status == STAT_CRC_OK){
				corrupt_pkt_count=0;

				for (j = 0; j < p->size; ++j) {
					//printf("%c",p->payload[j]);
					/*to copy the payload to recvBuff*/
				    recvBuff[j]=p->payload[j];
				}
				/* Create a socket first */
				if((sockfd = socket(AF_INET, SOCK_STREAM, 0))< 0){
				    printf("\n Error : Could not create socket \n");
				    return 1;
				}
				/* Attempt a connection */
				if(connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr))<0){
					printf("\n Error : Connect Failed \n");
					return 1;
				}
			    /* If read was success, send data. */
			    if(p->size > 0){
			    	printf("Sending \n");
			    	write(sockfd, recvBuff, p->size);
			    }
			    /*close socket*/
			    close(sockfd);
			} else {
				corrupt_pkt_count++;
				if (corrupt_pkt_count==10){
					printf("restart 0x10");
				}
			}
		}
	if (exit_sig == 1) {
		/* clean up before leaving */
		i = lgw_stop();
		if (i == LGW_HAL_SUCCESS) {
			MSG("INFO: concentrator stopped successfully\n");
		} else {
			MSG("WARNING: failed to stop concentrator successfully\n");
		}
		fclose(log_file);
		MSG("INFO: log file %s closed, %lu packet(s) recorded\n", log_file_name, pkt_in_log);
	}
	
	MSG("INFO: Exiting packet logger program\n");
	return EXIT_SUCCESS;
}
/* --- EOF ------------------------------------------------------------------ */
