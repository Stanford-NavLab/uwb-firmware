struct Complex {
	double re, im;
	double (*abss)(struct Complex *this);
};
extern const struct ComplexClass {
	struct Complex (*new)(double real, double imag);
} Complex;
