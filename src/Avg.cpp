/********************************************************************************
Copyright (c) 2017, Kolja Poreski
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the TAMS and HANDARBEITS-HAUS Poreski KG nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL KOLJA PORESKI BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/
#include <turtlebot_visual_docking/Avg.h> 

Avg::Avg(int _MESS){

MESS 			= _MESS;
avg_counter 		= -1;
avg_sum 		= 0.0;
avg_direction_angle 	= 0.0;
avg_ok 			= false;
	
avg_values		= new float[MESS];
for(int z = 0; z < MESS; z++){
		avg_values[z] = 0.0;
    }

}



/**
 * This function adds an value to an array.
 * When the array is full, the first element is deleted, then all 
 * elements are shifted one index to the left and the new element is 
 * added at the end.
 */
void Avg::new_value(float value){
	
	if(avg_counter < MESS){
		avg_counter++;
		avg_values[avg_counter] = value;
		avg_sum = avg_sum + value; 
		//	ROS_ERROR("%i --> %f",avg_counter,value);	
	}else{
		
		avg_sum = avg_sum - avg_values[0];

		float *new_values = new float[MESS]();
		for(int z = 0; z < MESS; z++){
			new_values[z]=0;
		}

		for(int i = 1; i < MESS; i++){
			new_values[i-1]=avg_values[i];
		}

		new_values[MESS-1]=value;
		for(int i = 0; i < MESS; i++){
			avg_values[i]=new_values[i];
		}
		avg_sum = avg_sum + value;
	}
	
}

/**
 * This function computes the mean of the value array.
 */
float Avg::avg(){
	if(avg_counter != -1){
		return avg_sum/(avg_counter + 1);
        }else{
                return 0;
        }
}


void Avg::flush_array(){
	avg_counter = -1;
	avg_sum     = 0.0;
}



/*
int main(int argc, char **argv){
 
    Avg avg;
    float *values = new float[100];
    float x = 300;
    float y = 300;
    for(int z = 0; z < 100; z++){
                float f = atan(x/y);
		x = x - (3*(float)z);
		y = y - ((float)z *0.5);
    		//printf("( %f , %f )-> %f \n",x,y,(float)atan(x/y));
		avg.new_value((180/M_PI)*(float)atan(x/y));
        	float Mittelwert = avg.avg();
        	printf("Mittelwert = %f \n",Mittelwert);
		x = 300;
		y = 300;
	}
}
*/
