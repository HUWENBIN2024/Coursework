 // SAMPLE_SOLUTION
#ifndef PARTICLE_H
#define PARTICLE_H

#include "vec.h"
#include <vector>
#include <algorithm>

class Particle {
public:
	Particle(Vec3d x, Vec3d v);
	Particle(const Particle& other);

	virtual void draw();
	virtual void update(double dt);
	Vec3d getx() const { return x; }
	Vec3d getv() const { return v; }
	friend class HairParticleSystem;

protected:
	Vec3d x; // position
	Vec3d v; // velocity
	Vec3d prev_v; // For air resistance too large detection
	vector<Vec3d> F; // all forces
	double m; // mass
	double r; // radius of sphere
	double c = 1; // air resistance coefficient
	virtual Vec3d sumF();
};

class SpringParticle : public Particle {
public:
	SpringParticle(Vec3d x, bool isStartPt = false, bool heavier = false);
	virtual void draw();
	virtual void update(double dt);
	virtual void updateSpringForce();
	void drawMass();
	void addNeighbor(SpringParticle* n);
	Vec3d SpringForce(SpringParticle* other, double L, double k);
	Vec3d computeMidpt(double dt);
	friend class HairParticleSystem;
protected:
	bool isStartPt = false;
private:
	bool heavier = false;
	vector<SpringParticle*> neighbors;
};

class ClothParticle : public SpringParticle {
public:
	ClothParticle(Vec3d x, bool isStartPt=false);
	virtual void update(double dt);
	void addNeighbor(ClothParticle* n, int type);
private:
	vector<ClothParticle*> neighbors0; // edge
	vector<ClothParticle*> neighbors1; // corner
	vector<ClothParticle*> neighbors2; // farther ones
};

class HairParticle : public SpringParticle {
public:
	HairParticle(Vec3d x, bool isStartPt = false);
	void addNeighbor(HairParticle* n, int type);
	friend class HairParticleSystem;
private:
	vector<HairParticle*> neighbors0; // closest
	vector<HairParticle*> neighbors1; // second closest
};

class FlockParticle : public Particle {
public:
	FlockParticle(Vec3d x) : Particle(x, Vec3d(0,0,0)) {}
	void updatex(double dt) { this->x += this->v * dt; }
	void updatev(Vec3d v) { this->v = v; }
};

bool operator==(const Particle& self, const Particle& other);

#endif // PARTICLE_H