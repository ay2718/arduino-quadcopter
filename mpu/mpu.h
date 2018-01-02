#ifndef mpu_h
#define mpu_h

class Vector{
public:
	Vector();
	float x, y, z;
	void rotate(Vector&);  //rotate infinitesimally
	float dot(Vector&);  //dot product
	Vector cross(Vector&);
	void add(Vector&);
	void multiply(float);
private:
	void normalize();
};

class mpu{
public:
    mpu();
	void mpuBegin();
    void readAccels();
	void readGyros();
	void compute();
	Vector rates;
    Vector gravity;
private:
    Vector accels;
	Vector gyros;
	Vector gyro_corr;
};



#endif // mpu_h
