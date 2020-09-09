#include <stdio.h>
#include <stdlib.h>


int main()
{
    double *array = (double*) malloc( (size_t) 257*257*257 * sizeof(double));

    FILE *fr = fopen("data/v-0.bin", "rb");  
    fread(array, sizeof(double)*257*257*257, 1, fr); 
    fclose(fr);

    FILE *fw = fopen("data/v-1.bin", "w+");
    for(long long i=0; i<(long long)257*257*257; i++) 
    {
	    float fval = (float) array[i];
	    fprintf(fw, "%f\n", fval);
    }
    fclose(fw);
}
