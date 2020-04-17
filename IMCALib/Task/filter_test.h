#ifndef ___FILTER_TEST
#define ___FILTER_TEST



typedef struct{
	double raw_value;
	double xbuf[18];
	double ybuf[18];
	double filtered_value;
}Filter_t;

extern Filter_t filter_test;


double Chebyshev50HzLPF(Filter_t *F, float in);


#endif
