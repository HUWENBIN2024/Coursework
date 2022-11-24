 // SAMPLE_SOLUTION
#include "particle.h"
#include "particleSystem.h"
#include "modelerdraw.h"
#include "modelerglobals.h"
#include "modelerapp.h"

bool operator==(const Particle& self, const Particle& other) {
	return self.getx() == other.getx(); // it's highly unusual when x is equal but particle is not
}

Particle::Particle(Vec3d x, Vec3d v) {
	this->x = x;
	this->v = v;
	this->m = 0.001;
	this->r = 0.03;
	this->prev_v = v;
	F.push_back(Vec3d(0, -9.8*m, 0)); // gravity
	F.push_back(v * (-c * r * r * v.length())); // air resistance
}

Particle::Particle(const Particle& other_p) {
	x = other_p.x;
	v = other_p.v;
	F = other_p.F;
	m = other_p.m;
	r = other_p.r;
	prev_v = other_p.prev_v;
}

void Particle::draw() {
	glPushMatrix();
	glTranslated(x[0], x[1], x[2]);
	setAmbientColor(0.6, 0.75, 0.3);
	drawSphere(r);
	setAmbientColor(0.1, 0.1, 0.1);
	glPopMatrix();
}

void Particle::update(double dt) {
	if (VAL(RK)) {
		Vec3d midptv = this->v + 0.5 * dt / m * (F[0] + F[1]);
		this->x += dt * midptv;
		this->v += dt / m * (F[0] + F[1]);
		// Air resistance cannot make v change to the opposite direction horizontally
		if (v[0] * prev_v[0] + v[2] * prev_v[2] < 0) {
			v[0] = 0;
			v[2] = 0;
		}
		this->prev_v = v;
		this->F.pop_back();
		this->F.push_back(v * (-c * r * r * v.length()));
	}
	else {
		this->x += dt * v;
		this->v += dt / m * (F[0] + F[1]);
		// Air resistance cannot make v change to the opposite direction horizontally
		if (v[0] * prev_v[0] + v[2] * prev_v[2] < 0) {
			v[0] = 0;
			v[2] = 0;
		}
		this->prev_v = v;
		this->F.pop_back();
		this->F.push_back(v * (-c * r * r * v.length()));
	}
}

Vec3d Particle::sumF() {
	Vec3d sum(0, 0, 0);
	vector<Vec3d>::iterator iter;
	for (iter = F.begin(); iter != F.end(); ++iter) {
		sum += *iter;
	}
	return sum;
}

SpringParticle::SpringParticle(Vec3d x, bool isStartPt, bool heavier)
	: Particle(x, Vec3d(0, 0, 0)), isStartPt(isStartPt), heavier(heavier) {
	if (heavier) {
		m = 0.1;
		r = 0.14;
		c = 0;
	}
	else {
		c = 30;
	}
	F.clear();
	F.push_back(Vec3d(0, -9.8 * m, 0)); // gravity
	F.push_back(Vec3d(0, 0, 0)); // air resistance
}

void SpringParticle::draw() {
	glPushMatrix();
	glTranslated(x[0], x[1], x[2]);
	setAmbientColor(0, 1, 1);
	setDiffuseColor(0, 1, 1);
	drawSphere(2*r);
	SpringParticle* upper = nullptr;
	if (!(neighbors.empty())) upper = neighbors[0];
	if (upper) {
		Vec3d direction = upper->x - this->x; double h = direction.length(); direction.normalize();
		Vec3d axis = direction + Vec3d(0, 0, 1); axis.normalize();
		glRotated(180, axis[0], axis[1], axis[2]);
		setAmbientColor(1, 1, 1);
		setDiffuseColor(1, 1, 1);
		drawCylinder(h, 0.02, 0.02);
	}
	glPopMatrix();
}

void SpringParticle::drawMass() {
	glPushMatrix();
	glTranslated(x[0], x[1], x[2]);
	setAmbientColor(0, 1, 0);
	setDiffuseColor(0, 1, 0);
	drawSphere(2 * r);
	glPopMatrix();
}

void SpringParticle::updateSpringForce() {
	SpringParticleSystem* ps = dynamic_cast<SpringParticleSystem*>(ModelerApplication::Instance()->GetParticleSystem());
	double L = ps->getRestLength(), k = ps->getk();

	while (F.size() > 1) this->F.pop_back();
	Vec3d springNetForce(0, 0, 0);
	vector<SpringParticle*>::iterator iter;
	for (iter = neighbors.begin(); iter != neighbors.end(); ++iter) {
		springNetForce += SpringForce(*iter,L,k);
	}
	this->F.push_back(springNetForce);
}

void SpringParticle::update(double dt) {
	if (isStartPt) return;

	this->x += dt * v;

	while (F.size() > 2) this->F.pop_back();
	this->F.push_back(v * (-c * r * r * v.length()));

	this->v += dt / m * sumF();
}

void SpringParticle::addNeighbor(SpringParticle* neighbor) {
	neighbors.push_back(neighbor);
}

// spring force applied on "this" from another particle
Vec3d SpringParticle::SpringForce(SpringParticle* other, double L, double k) {
	Vec3d direction = (other->x - this->x); double norm = direction.length(); direction.normalize();
	return k * (norm - L) * direction;
}

Vec3d SpringParticle::computeMidpt(double dt) {
	return this->x + 0.5 * dt * v;
}

ClothParticle::ClothParticle(Vec3d x, bool isStartPt) : SpringParticle(x,isStartPt,false) {
	m = 0.01;
	c = 0.6;
	F.clear();
	F.push_back(Vec3d(0, -9.8 * m, 0));
	F.push_back(Vec3d(0, 0, 0)); // air resistance
}

void ClothParticle::update(double dt) {
	if (isStartPt) return;

	Vec3d prev_x = x;

	Vec3d midptv = this->v + dt / m * sumF() / 2;
	while (F.size() > 1) this->F.pop_back();
	this->F.push_back(midptv * (-c * r * r * midptv.length()));

	Vec3d midptx = prev_x + 0.5 * dt * midptv;
	x = midptx;

	ClothParticleSystem* ps = dynamic_cast<ClothParticleSystem*>(ModelerApplication::Instance()->GetParticleSystem());
	double* L = ps->getL(), * K = ps->getK();

	while (F.size() > 2) this->F.pop_back();
	Vec3d springNetForce(0, 0, 0);
	vector<ClothParticle*>::iterator iter;
	for (iter = neighbors0.begin(); iter != neighbors0.end(); ++iter) {
		springNetForce += SpringForce(*iter, L[0], K[0]);
	}
	for (iter = neighbors1.begin(); iter != neighbors1.end(); ++iter) {
		springNetForce += SpringForce(*iter, L[1], K[1]);
	}
	for (iter = neighbors2.begin(); iter != neighbors2.end(); ++iter) {
		springNetForce += SpringForce(*iter, L[2], K[2]);
	}
	this->F.push_back(springNetForce);

	this->v += dt / m * sumF();

	this->x = prev_x + dt * v;
}

void ClothParticle::addNeighbor(ClothParticle* n, int type) {
	if (!n) return;
	switch (type) {
	case 0: neighbors0.push_back(n); break;
	case 1: neighbors1.push_back(n); break;
	case 2: neighbors2.push_back(n); break;
	default: cerr << "bad type index for adding cloth particle neighbors" << endl;
	}
}

HairParticle::HairParticle(Vec3d x, bool isStartPt) : SpringParticle(x, isStartPt, false) {
	m = 0.001;
	c = 2;
	F.clear();
	F.push_back(Vec3d(0, -9.8 * m, 0)); // gravity
	F.push_back(Vec3d(0, 0, 0)); // air resistance
}

void HairParticle::addNeighbor(HairParticle* n, int type) {
	if (!n) return;
	switch (type) {
	case 0: neighbors0.push_back(n); break;
	case 1: neighbors1.push_back(n); break;
	default: cerr << "bad type index for adding cloth particle neighbors" << endl;
	}
}