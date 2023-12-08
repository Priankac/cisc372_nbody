#include <stdlib.h>
#include <math.h>
#include "vector.h"
#include "config.h"

//compute: Updates the positions and locations of the objects in the system based on gravity.
//Parameters: None
//Returns: None
//Side Effect: Modifies the hPos and hVel arrays with the new positions and accelerations after 1 INTERVAL
// Function to calculate the pairwise acceleration matrix
void AccelerationMatrix() {
    vector3* accels = (vector3*)malloc(sizeof(vector3) * NUMENTITIES * NUMENTITIES);

    for (int i = 0; i < NUMENTITIES; ++i) {
        for (int j = 0; j < NUMENTITIES; ++j) {
            if (i == j) {
                // diagonal elements zero acceleration
                FILL_VECTOR(accels[i * NUMENTITIES + j], 0, 0, 0);
            } else {
                // distance between objects i and j
                vector3 distance;
                for (int k = 0; k < 3; ++k) {
                    distance[k] = hPos[i][k] - hPos[j][k];
                }
                double sq = distance[0] * distance[0] + distance[1] * distance[1] + distance[2] * distance[2];
				double acceler = -1 * GRAV_CONSTANT * mass[j] / sq;
                double magnitudes = sqrt(sq);

                //acceleration components
                double ax = acceler * distance[0] / magnitudes;
                double ay = acceler * distance[1] / magnitudes;
                double az = acceler * distance[2] / magnitudes;

                FILL_VECTOR(accels[i * NUMENTITIES + j], ax, ay, az);
            }
        }
    }
}

void Acceleration(vector3* accels, vector3* hPos, vector3* hVel, double* mass) {
    vector3 accel_sum[NUMENTITIES];
    
  
    for (int i = 0; i < NUMENTITIES; ++i) {
        for (int j = 0; j < NUMENTITIES; ++j) {
            accel_sum[i][0] += accels[j * NUMENTITIES + i][0];
            accel_sum[i][1] += accels[j * NUMENTITIES + i][1];
            accel_sum[i][2] += accels[j * NUMENTITIES + i][2];
        }
    }

    
    for (int i = 0; i < NUMENTITIES; ++i) {
        hPos[i][0] += hVel[i][0] * INTERVAL;
        hPos[i][1] += hVel[i][1] * INTERVAL;
        hPos[i][2] += hVel[i][2] * INTERVAL;

		hVel[i][0] += accel_sum[i][0] * INTERVAL;
        hVel[i][1] += accel_sum[i][1] * INTERVAL;
        hVel[i][2] += accel_sum[i][2] * INTERVAL;
    }
}

void compute(){
	//make an acceleration matrix which is NUMENTITIES squared in size;
	int i,j,k;
	vector3* values=(vector3*)malloc(sizeof(vector3)*NUMENTITIES*NUMENTITIES);
	vector3** accels=(vector3**)malloc(sizeof(vector3*)*NUMENTITIES);
	for (i=0;i<NUMENTITIES;i++)
		accels[i]=&values[i*NUMENTITIES];
	//first compute the pairwise accelerations.  Effect is on the first argument.
	for (i=0;i<NUMENTITIES;i++){
		for (j=0;j<NUMENTITIES;j++){
			if (i==j) {
				FILL_VECTOR(accels[i][j],0,0,0);
			}
			else{
				vector3 distance;
				for (k=0;k<3;k++) distance[k]=hPos[i][k]-hPos[j][k];
				double magnitude_sq=distance[0]*distance[0]+distance[1]*distance[1]+distance[2]*distance[2];
				double magnitude=sqrt(magnitude_sq);
				double accelmag=-1*GRAV_CONSTANT*mass[j]/magnitude_sq;
				FILL_VECTOR(accels[i][j],accelmag*distance[0]/magnitude,accelmag*distance[1]/magnitude,accelmag*distance[2]/magnitude);
			}
		}
	}
	//sum up the rows of our matrix to get effect on each entity, then update velocity and position.
	for (i=0;i<NUMENTITIES;i++){
		vector3 accel_sum={0,0,0};
		for (j=0;j<NUMENTITIES;j++){
			for (k=0;k<3;k++)
				accel_sum[k]+=accels[i][j][k];
		}
		//compute the new velocity based on the acceleration and time interval
		//compute the new position based on the velocity and time interval
		for (k=0;k<3;k++){
			hVel[i][k]+=accel_sum[k]*INTERVAL;
			hPos[i][k]+=hVel[i][k]*INTERVAL;
		}
	}
	free(accels);
	free(values);
}
