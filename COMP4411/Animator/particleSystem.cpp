#pragma warning(disable : 4786)

#include "particleSystem.h"
#include "modelerapp.h"
#include "modelerui.h"


#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>
#include <limits.h>

extern bool initialized;
#ifndef cpcount
#define cpcount 13
#endif
#ifndef hcount
#define hcount 12
#endif

//deep copy helper function
vector<Particle*> copy(vector<Particle*> ps) {
	vector<Particle*> new_ps;
	for (vector<Particle*>::iterator i = ps.begin(); i != ps.end(); ++i) {
		Particle* new_p = new Particle(**i);
		new_ps.push_back(new_p);
	}
	return new_ps;
}

/***************
 * Constructors
 ***************/

ParticleSystem::ParticleSystem() 
{
	// TODO
	simulate = false;
	dirty = false;
	this->TYPE = DEFAULT;
}

ParticleSystem::ParticleSystem(SampleParticleSystems TYPE) {
	simulate = false;
	dirty = false;
	this->TYPE = TYPE;
}

/*************
 * Destructor
 *************/

ParticleSystem::~ParticleSystem() 
{
	// TODO
	// Beware of memory leak
}


/******************
 * Simulation fxns
 ******************/

void ParticleSystem::add(Particle* p) {
	// TODO
	cur_particles.push_back(p);
}

/** Start the simulation */
void ParticleSystem::startSimulation(float t) // when "simulate" button is turned on
{
    
	// TODO

	// These values are used by the UI ...
	// -ve bake_end_time indicates that simulation
	// is still progressing, and allows the
	// indicator window above the time slider
	// to correctly show the "baked" region
	// in grey.
	bake_fps = ModelerApplication::Instance()->GetUI()->fps();
	bake_end_time = -1;
	simulate = true;
	dirty = true;

}

/** Stop the simulation */
void ParticleSystem::stopSimulation(float t) // when "simulate" button is turned off
{
    
	// TODO
	bake_end_time = ModelerApplication::Instance()->GetUI()->playEndTime();

	// These values are used by the UI
	simulate = false;
	dirty = true;

}

/** Reset the simulation */
void ParticleSystem::resetSimulation(float t)
{
    
	// TODO

	// These values are used by the UI
	simulate = false;
	dirty = true;

}

/** Compute forces and update particles **/
void ParticleSystem::computeForcesAndUpdateParticles(float t)
{
	// TODO
	// t is current time; need to know previous state of particles (which is current particle state at this stage)
	// if baked at t: update current particle state to be the baked one, then return
	// else: update current particle state to be the computed result, and bake
	if (bake_map.count(t)) {
		cur_particles = copy(bake_map.find(t)->second);
	}
	else {
		for (vector<Particle*>::iterator i = cur_particles.begin(); i != cur_particles.end(); ++i) {
			double dt = 1 / bake_fps;
			dt /= VAL(EULER_PRECISION);
			for (int j = 0; j < VAL(EULER_PRECISION); ++j) {
				(*i)->update(dt);
			}
		}
		bakeParticles(t);
	}

	// Destroy Useless Particles
	for (vector<Particle*>::iterator iter = cur_particles.begin(); iter != cur_particles.end();) {
		if ((*iter)->getx()[1] < (-WHEEL_RADIUS - 10) * SCALE_FACTOR) {
			delete (*iter);
			iter = cur_particles.erase(iter);
		}
		else {
			++iter;
		}
	}
}

/** Render particles */
void ParticleSystem::drawParticles(float t)
{
	// TODO
	// For particle in particles:
	//		get info of particle from current state. forget about baking since it's handled when computing.
	//		draw it
	for (vector<Particle*>::iterator i = cur_particles.begin(); i != cur_particles.end(); ++i) {
		(*i)->draw();
	}
}

/** Adds the current configuration of particles to
  * your data structure for storing baked particles **/
void ParticleSystem::bakeParticles(float t) 
{
	// TODO
	bake_map.insert(pair<float, vector<Particle*>>(t, copy(cur_particles)));
}

/** Clears out your data structure of baked particles */
void ParticleSystem::clearBaked() // when "clearSim" button is clicked
{
	// TODO
	for (map<float, vector<Particle*>>::iterator i = bake_map.begin(); i != bake_map.end(); ++i) {
		vector<Particle*> ps = i->second;
		for (vector<Particle*>::iterator j = ps.begin(); j != ps.end(); ++j) {
			delete (*j);
			(*j) = nullptr;
		}
	}
	bake_map.clear();
}

//========Spring Particle System Functions==========
SpringParticleSystem::SpringParticleSystem() {
	TYPE = SPRING;
	simulate = false;
	dirty = false;
}

void SpringParticleSystem::drawParticles(float t) {
	vector<Particle*>::iterator i;
	for (i = cur_particles.begin(); i != cur_particles.end(); ++i) {
		dynamic_cast<SpringParticle*>(*i)->draw();
	}
	if (!cur_particles.empty()) {
		--i;
		dynamic_cast<SpringParticle*>(*i)->drawMass();
	}
}

void SpringParticleSystem::computeForcesAndUpdateParticles(float t) {
	double dt = 1 / bake_fps;
	dt /= VAL(EULER_PRECISION);
	for (int j = 0; j < VAL(EULER_PRECISION); ++j) {
		//vector<Vec3d> midpts;
		vector<Particle*>::iterator i = cur_particles.begin();
		//for (i = cur_particles.begin(); i != cur_particles.end(); ++i) {
		//	SpringParticle* sp = dynamic_cast<SpringParticle*>(*i);
		//	//midpts.push_back(sp->computeMidpt(dt));
		//}
		for (i = cur_particles.begin(); i != cur_particles.end(); ++i) {
			SpringParticle* sp = dynamic_cast<SpringParticle*>(*i);
			sp->updateSpringForce();
			sp->update(dt);
		}
	}
}

void SpringParticleSystem::clearBaked() {
	vector<Particle*>::iterator i = cur_particles.begin();
	for (i = cur_particles.begin(); i != cur_particles.end(); ++i) {
		delete dynamic_cast<SpringParticle*>(*i);
	}
	cur_particles.clear();
	initialized = false;
}

//==========cloth particle system functions==========
ClothParticleSystem::ClothParticleSystem() {
	TYPE = CLOTH;
	simulate = false;
	dirty = false;
}

void ClothParticleSystem::drawParticles(float t) {
	if (cur_particles.empty()) return;
	const int n = cpcount;
	ClothParticle* grid[n][n] = { nullptr };
	for (int i = 0; i < cur_particles.size(); ++i) {
		grid[0][i] = dynamic_cast<ClothParticle*>(cur_particles[i]);
	}
	for (int j = 0; j < n - 1; ++j) {
		for (int i = 0; i < n - 1; ++i) {
			Vec3d zs = grid[j][i]->getx();
			Vec3d zx = grid[j + 1][i]->getx();
			Vec3d ys = grid[j][i+1]->getx();
			Vec3d yx = grid[j + 1][i + 1]->getx();
			Vec3d avg = (zs+zx+ys+yx) / 4;
			setDiffuseColor(1, 0, 1);
			drawTriangle(zs, zx, avg);
			drawTriangle(zx, zs, avg);
			drawTriangle(zs, ys, avg);
			drawTriangle(ys, zs, avg);
			drawTriangle(yx, zx, avg);
			drawTriangle(zx, yx, avg);
			drawTriangle(ys, yx, avg);
			drawTriangle(yx, ys, avg);
		}
	}
}

void ClothParticleSystem::computeForcesAndUpdateParticles(float t) {
	double dt = 1 / bake_fps;
	dt /= VAL(EULER_PRECISION);
	for (int j = 0; j < VAL(EULER_PRECISION); ++j) {
		vector<Particle*>::iterator i = cur_particles.begin();
		for (i = cur_particles.begin(); i != cur_particles.end(); ++i) {
			ClothParticle* sp = dynamic_cast<ClothParticle*>(*i);
			sp->update(dt);
		}
	}
}

void ClothParticleSystem::clearBaked() {
	vector<Particle*>::iterator i = cur_particles.begin();
	for (i = cur_particles.begin(); i != cur_particles.end(); ++i) {
		delete dynamic_cast<ClothParticle*>(*i);
	}
	cur_particles.clear();
	initialized = false;
}

//========Hair Particle System functions========
HairParticleSystem::HairParticleSystem() {
	TYPE = HAIR;
	simulate = false;
	dirty = false;
}

void HairParticleSystem::drawParticles(float t) {
	if (cur_particles.empty()) return;
	const int n = hcount;
	HairParticle* grid[n/3][n][18] = { nullptr };
	for (int i = 0; i < cur_particles.size(); ++i) {
		grid[0][0][i] = dynamic_cast<HairParticle*>(cur_particles[i]);
	}
	for (int j = 0; j < n/3; ++j) {
		for (int i = 0; i < n; ++i) {
			for (int k = 0; k < 18-1; ++k) {
				glPushMatrix();
				Vec3d x = grid[j][i][k]->getx();
				glTranslated(x[0], x[1], x[2]);
				Vec3d dir = grid[j][i][k + 1]->getx() - x; double L = dir.length();  dir.normalize();
				Vec3d axs = dir + Vec3d(0, 0, 1); axs.normalize();
				glRotated(180, axs[0], axs[1], axs[2]);
				setDiffuseColor(0, 1, 0);
				drawCylinder(L, 0.01, 0.01);
				glPopMatrix();
			}
		}
	}
}

void HairParticleSystem::computeForcesAndUpdateParticles(float t) {
	double dt = 1 / bake_fps;
	dt /= VAL(EULER_PRECISION);
	for (int _ = 0; _ < VAL(EULER_PRECISION); ++_) {
		int size = cur_particles.size(); int i;

		const int n = hcount;
		Vec3d prev_x[n/3*n*18];
		for (i = 0; i < size; ++i) {
			HairParticle* sp = dynamic_cast<HairParticle*>(cur_particles[i]);
			if (sp->isStartPt) continue;
			prev_x[i] = sp->x;
		}
		for (i = 0; i < size; ++i) {
			HairParticle* sp = dynamic_cast<HairParticle*>(cur_particles[i]);
			if (sp->isStartPt) continue;
			Vec3d midptv = sp->v + dt * sp->sumF() / sp->m / 2;
			while (sp->F.size() > 1) sp->F.pop_back();
			sp->F.push_back(midptv * (-sp->c * sp->r * sp->r * midptv.length()));
			Vec3d midptx = prev_x[i] + 0.5 * dt * midptv;
			sp->x = midptx;
		}
		for (i = 0; i < size; ++i) {
			HairParticle* sp = dynamic_cast<HairParticle*>(cur_particles[i]);
			if (sp->isStartPt) continue;

			while (sp->F.size() > 2) sp->F.pop_back();
			Vec3d springNetForce(0, 0, 0);
			vector<HairParticle*>::iterator iter;
			for (iter = sp->neighbors0.begin(); iter != sp->neighbors0.end(); ++iter) {
				springNetForce += sp->SpringForce(*iter, L[0], k[0]);
			}
			for (iter = sp->neighbors1.begin(); iter != sp->neighbors1.end(); ++iter) {
				springNetForce += sp->SpringForce(*iter, L[1], k[1]);
			}
			if (VAL(WIND)) {
				double windf = pow(min(0,sp->x[1]),2);
				springNetForce += Vec3d(-0.005*windf,0,-0.01*windf);
			}
			sp->F.push_back(springNetForce);

			sp->v += dt / sp->m * sp->sumF();
		}
		for (i = 0; i < size; ++i) {
			HairParticle* sp = dynamic_cast<HairParticle*>(cur_particles[i]);
			if (sp->isStartPt) continue;
			sp->x = prev_x[i] + dt * sp->v;
			if (sp->x[2] < -0.5 && VAL(WIND)==1) sp->x[2] = -0.5;
		}
		for (i = 0; i < size; ++i) {
			HairParticle* sp = dynamic_cast<HairParticle*>(cur_particles[i]);
			if (sp->isStartPt) continue;

			while (sp->F.size() > 2) sp->F.pop_back();
			Vec3d springNetForce(0, 0, 0);
			vector<HairParticle*>::iterator iter;
			for (iter = sp->neighbors0.begin(); iter != sp->neighbors0.end(); ++iter) {
				springNetForce += sp->SpringForce(*iter, L[0], k[0]);
			}
			for (iter = sp->neighbors1.begin(); iter != sp->neighbors1.end(); ++iter) {
				springNetForce += sp->SpringForce(*iter, L[1], k[1]);
			}
			sp->F.push_back(springNetForce);
		}
	}
}

void HairParticleSystem::clearBaked() {
	vector<Particle*>::iterator i = cur_particles.begin();
	for (i = cur_particles.begin(); i != cur_particles.end(); ++i) {
		delete dynamic_cast<HairParticle*>(*i);
	}
	cur_particles.clear();
	initialized = false;
}

//===========Flock Particle System============
FlockParticleSystem::FlockParticleSystem() {
	TYPE = FLOCK;
	simulate = false;
	dirty = false;
}

void FlockParticleSystem::drawParticles(float t) {
	for (vector<Particle*>::iterator i = cur_particles.begin(); i != cur_particles.end(); ++i) {
		glPushMatrix();
		Vec3d x = (*i)->getx();
		glTranslated(x[0], x[1], x[2]);
		glTranslated(0, 0.1, 0);
		Vec3d dir = (*i)->getv(); double L = dir.length(); dir.normalize();
		Vec3d axs = dir + Vec3d(0, 0, 1); axs.normalize();
		glRotated(180, axs[0], axs[1], axs[2]);
		setDiffuseColor(0, 1, 1);
		drawCylinder(0.2, 0.1, 0);
		glPopMatrix();
	}
}

void FlockParticleSystem::computeForcesAndUpdateParticles(float t) {
	double dt = 1 / bake_fps;
	dt /= VAL(EULER_PRECISION);
	for (int _ = 0; _ < VAL(EULER_PRECISION); ++_) {
		double rad = VAL(FLOCKTARGET) / 180 * M_PI;
		targetx = Vec3d(6 * cos(rad), 0, 6 * sin(rad));
		int size = cur_particles.size(); int i;
		// update v
		for (i = 0; i < size; ++i) {
			FlockParticle* sp = dynamic_cast<FlockParticle*>(cur_particles[i]);
			// v towards target
			Vec3d v0 = targetx - sp->getx(); v0.normalize(); v0 *= r0;
			// v towards center of neighbors
			Vec3d center(0, 0, 0); int count = 0;
			for (vector<Particle*>::iterator j = cur_particles.begin(); j != cur_particles.end(); ++j) {
				if ((sp->getx() - (*j)->getx()).length() < localR && sp != *j) {
					Vec3d d1 = (*j)->getx() - sp->getx(); d1.normalize();
					Vec3d d2 = v0; d2.normalize();
					if (d1 * d2 > -1) {
						center += (*j)->getx();
						++count;
					}
				}
			}
			center /= count;
			Vec3d v1 = center - sp->getx(); v1.normalize(); v1 *= r1;
			if (count == 0) v1 = Vec3d(0, 0, 0);
			// v pushed by neighbors
			Vec3d v2(0, 0, 0);
			for (vector<Particle*>::iterator j = cur_particles.begin(); j != cur_particles.end(); ++j) {
				if ((sp->getx() - (*j)->getx()).length() < localR && sp != *j) {
					Vec3d d1 = (*j)->getx() - sp->getx(); d1.normalize();
					Vec3d d2 = v0; d2.normalize();
					if (d1 * d2 > -1) {
						Vec3d direction = sp->getx() - (*j)->getx(); direction.normalize(); direction *= r2;
						v2 += direction;
					}
				}
			}
			// do update
			sp->updatev(v0 + v1 + v2);
		}
		// update x
		for (i = 0; i < size; ++i) {
			FlockParticle* sp = dynamic_cast<FlockParticle*>(cur_particles[i]);
			sp->updatex(dt);
		}
		// disappear if fall outside
		for (vector<Particle*>::iterator i = cur_particles.begin(); i != cur_particles.end();) {
			FlockParticle* sp = dynamic_cast<FlockParticle*>(*i);
			if (sp->getx().length() > 6) {
				delete sp;
				i = cur_particles.erase(i);
			}
			else {
				++i;
			}
		}
	}
}

void FlockParticleSystem::clearBaked() {
	vector<Particle*>::iterator i = cur_particles.begin();
	for (i = cur_particles.begin(); i != cur_particles.end(); ++i) {
		delete dynamic_cast<FlockParticle*>(*i);
	}
	cur_particles.clear();
}