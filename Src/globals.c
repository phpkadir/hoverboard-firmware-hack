#include <stdint.h>
#include "comms.h"

volatile uint8_t cache_data[1024];

inline void swp(int* x,int* y){
	int tmp = *x;
	*x = *y;
	*y = tmp;
}
inline void sort_array(int x[],int cnt){
  for(int y=0;y<cnt-1; y++)
		for(int z=y+1;z<cnt; z++)
			if(x[y]>x[z])
				swp(&x[y],&x[z]);
}
int calc_median(int x[],int cnt){
  sort_array(x,cnt);
	if(cnt%2)
		return x[cnt/2];
	else
		return (x[cnt/2]+x[cnt/2+1])/2;
}

int32_t get_phase(){
    int32_t* cache_data32 = &cache_data;
    int x = 0;
    for(uint8_t board = 0; board< 2; board++)
        for(uint8_t b = 0; b< 2; b++, x++)
            cache_data32[x] = virutal_phase[board][b];
    return calc_median(cache_data32,x);
}