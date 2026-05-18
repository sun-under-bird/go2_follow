#ifndef _LIB_AOA_ALGO_H
#define _LIB_AOA_ALGO_H

#include<stdio.h>
#include<stdint.h>

struct output {
	float r; // m
	float rad; // rad [-pi,pi];
	float x; // m
	float y; // m
	int state; // > = 0 normal,-1 wrong, -2 wrong distance, -3 wrong angle
};

struct input_data {
	// Measured Data
	int State; // Master get data or not 0:no 1:yes
	float Distance; // Master distance measured m
	float Azimuth; // Master angle measured deg [-180,180]
	float Pitch; // pitch measured deg [0,90]
	uint32_t t; // Master time stamp ms


	// Base Info
	float StationXOffset; // Master station pos (x,y) m
	float StationYOffset;
	float config_r; // Master distanceOffset cm
	float config_rad; // Master angleOffset deg [-180,180]
	int direction; // Master clockwise direction 1 / -1

	// Fillter Info:
	int win_fil; // filter window, <= 20;
	int use_pitch_dis; // dis<use_pitch_dis -> use pitch, m
	float stopband_rate; // stopband rate for filter, 0-0.4

	// Clean Buffer Info
	int clean_buffer_time; // clean buffer data when base stop, s
	int aoa_frequency; // fob/phone test frequence, Hz
};

unsigned int algo_aoa_get_ver(void);
unsigned char algo_uwb_aoa_get_ver_patch(void);
unsigned int algo_uwb_aoa_get_ver_all(void);
int algo_uwb_aoa_get_ver_str(unsigned char* ver_str, unsigned char len);

void algo_uwb_aoa_clean(void);
unsigned char algo_uwb_aoa_merge(struct input_data* input_rawdata, struct output* out_mergedata);

#endif
