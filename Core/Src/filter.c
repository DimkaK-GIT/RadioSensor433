#include "filter.h"

filterHandle filter(filterHandle data, float in)
{
	filterHandle result;
	float summ = 0;
	int i;
	float mind,maxd;
	
	for(i=0;i<4;i++)
	  result.fillbuff[4-i] = data.fillbuff[3-i];
	result.fillbuff[0] = in;
	
	result.count = data.count + 1;
	
	if(result.count < 5)
		return result;

	if(result.count > 200)
			result.count --;
	


	mind = result.fillbuff[0];
	maxd = result.fillbuff[0];
	summ = result.fillbuff[0];
	
	for(i=1;i<5;i++)
	{
	  if(mind > result.fillbuff[i])
       mind = result.fillbuff[i];

	  if(maxd < result.fillbuff[i])
       maxd = result.fillbuff[i];

		summ = summ + result.fillbuff[i];
	}

	summ -= mind;
	summ -= maxd;
	
	result.result = summ / 3.0;
	return result;
}

//