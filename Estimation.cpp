
const float rad_per_deg = M_PI / 180.0;

static float innerProduct(Vector v1,Vector v2){
	return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
}

static Vector outerProduct(Vector v1,Vector v2){
	return (Vector){
		v1.y*v2.z - v1.z*v2.y,
		v1.z*v2.x - v1.x*v2.z,
		v1.x*v2.y - v1.y*v2.x
	};
}

static Quaternion generateCorrecor(Vector world_v,Vector goal_v,float alpha){
		Vector outerP = outerProduct(world_v, goal_v);
		float absolute_outerP = outerP.absolute();
		Vector e = {
			outerP.x / absolute_outerP,
			outerP.y / absolute_outerP,
			outerP.z / absolute_outerP };
		float theata = acosf(innerProduct(world_v, goal_v) / (world_v.absolute() * goal_v.absolute()));
		float Sin = sinf(theata * alpha * 0.5f);
		return (Quaternion){
			e.x * Sin,
			e.y * Sin,
			e.z * Sin,
			cosf(theata * alpha * 0.5f)
		};
}

static Vector toWorld(Vector v,Quaternion q){
	return (Vector){
	 	( q.x*q.x-q.y*q.y-q.z*q.z+q.w*q.w)*v.x + 2*(q.x*q.y+q.z*q.w)*v.y + 2*(q.x*q.z-q.y*q.w)*v.z,
	 	2*(q.x*q.y-q.z*q.w)*v.x + (-q.x*q.x+q.y*q.y-q.z*q.z+q.w*q.w)*v.y + 2*(q.y*q.z+q.x*q.w)*v.z,
	 	2*(q.x*q.z+q.y*q.w)*v.x + 2*(q.y*q.z-q.x*q.w)*v.y + (-q.x*q.x-q.y*q.y+q.z*q.z+q.w*q.w)*v.z
	};
	// q * v * q^-1
}


float Vector::absolute(){
	return sqrtf(x*x + y*y + z*z);
}

void AttitudeEstimator::Update(float GyrX,float GyrY,float GyrZ,float dt){
	//Gyr is AngularVelocity in (deg/s)
	//dt is delta time in seconds
	/*
	float sx = sinf(GyrX * rad_per_deg * 0.5f * dt)
	 	, sy = sinf(GyrY * rad_per_deg * 0.5f * dt)
	 	, sz = sinf(GyrZ * rad_per_deg * 0.5f * dt)
	 	, cx = cosf(GyrX * rad_per_deg * 0.5f * dt)
	 	, cy = cosf(GyrY * rad_per_deg * 0.5f * dt)
	 	, cz = cosf(GyrZ * rad_per_deg * 0.5f * dt);
	Quaternion dq = (Quaternion){
						sx*cy*cz + cx*sy*sz,
						cx*sy*cz - sx*cy*sz,
						sx*sy*cz + cx*cy*sz,
					   -sx*sy*sz - cx*cy*cz 	};
	//q = q.multi((Quaternion){sx,  0,  0, cx});
	//q = q.multi((Quaternion){ 0, sy,  0, cy});
	//q = q.multi((Quaternion){ 0,  0, sz, cz});
	q = q.multi(dq);
	*/
	
	// 近似積分
	// q = 0.5*dt*ω*q + q
	// 簡単に実装するために，角速度[rad/s]を純虚四元数として扱う
	Quaternion omega_pure = (Quaternion){
		rad_per_deg * GyroX,
		rad_per_deg * GyroY,
		rad_per_deg * GyroZ,
		0.0f
	};
	float coef = 0.5f * dt;
	Quaternion tmp = q.multi(omega_pure);  // ω*q
	q = (Quaternion){
		coef * tmp.x + q.x,
		coef * tmp.y + q.y,
		coef * tmp.z + q.z,
		coef * tmp.w + q.w
	};
	// 正規化
	float norm_inv = 1.0f / sqrtf(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
	q = (Quaternion){
		q.x * norm_inv,
		q.y * norm_inv,
		q.z * norm_inv,
		q.w * norm_inv
	};
	
	time += dt;
}

void AttitudeEstimator::Update(float GyrX,float GyrY,float GyrZ,float AccX,float AccY,float AccZ,float dt){
	Update(GyrX, GyrY, GyrZ, dt);
	Vector acc_world = toWorld((Vector){AccX,AccY,AccZ}, q);
	Quaternion c = generateCorrecor(acc_world, gravity, AccAlpha);
	q = q.multi(c);
}
