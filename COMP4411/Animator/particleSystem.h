/***********************
 * ParticleSystem class
 ***********************/

/**
 * The particle system class simply "manages" a collection of particles.
 * Its primary responsibility is to run the simulation, evolving particles
 * over time according to the applied forces using Euler's method.
 * This header file contains the functions that you are required to implement.
 * (i.e. the rest of the code relies on this interface)
 * In addition, there are a few suggested state variables included.
 * You should add to this class (and probably create new classes to model
 * particles and forces) to build your system.
 */

#ifndef __PARTICLE_SYSTEM_H__
#define __PARTICLE_SYSTEM_H__

#include "vec.h"
#include "particle.h"
#include "modelerglobals.h"
#include <map>
#include <vector>
#include <algorithm>
using namespace std;

class ParticleSystem {

public:



	/** Constructor **/
	ParticleSystem();
	ParticleSystem(SampleParticleSystems TYPE);


	/** Destructor **/
	virtual ~ParticleSystem();

	/** Simulation fxns **/
	// Add Particles to the system at a specific start point
	virtual void add(Particle* p);
	// This fxn should render all particles in the system,
	// at current time t.
	virtual void drawParticles(float t);

	// This fxn should save the configuration of all particles
	// at current time t.
	virtual void bakeParticles(float t);

	// This function should compute forces acting on all particles
	// and update their state (pos and vel) appropriately.
	virtual void computeForcesAndUpdateParticles(float t);

	// This function should reset the system to its initial state.
	// When you need to reset your simulation, PLEASE USE THIS FXN.
	// It sets some state variables that the UI requires to properly
	// update the display.  Ditto for the following two functions.
	virtual void resetSimulation(float t);

	// This function should start the simulation
	virtual void startSimulation(float t);

	// This function should stop the simulation
	virtual void stopSimulation(float t);

	// This function should clear out your data structure
	// of baked particles (without leaking memory).
	virtual void clearBaked();	



	// These accessor fxns are implemented for you
	float getBakeStartTime() { return bake_start_time; }
	float getBakeEndTime() { return bake_end_time; }
	float getBakeFps() { return bake_fps; }
	SampleParticleSystems getType() { return TYPE; }
	vector<Particle*>& getCurParticles() { return cur_particles; }
	void setBakeFps(float fps) { bake_fps = fps; }
	bool isSimulate() { return simulate; }
	bool isDirty() { return dirty; }
	void setDirty(bool d) { dirty = d; }



protected:
	SampleParticleSystems TYPE;
	
	vector<Particle*> cur_particles; // all particles
	map<float, vector<Particle*>> bake_map; // store baked particle information

	/** Some baking-related state **/
	float bake_fps;						// frame rate at which simulation was baked
	float bake_start_time;				// time at which baking started 可能没用
										// These 2 variables are used by the UI for
										// updating the grey indicator 
	float bake_end_time;				// time at which baking ended

	/** General state variables **/
	bool simulate;						// flag for simulation mode
	bool dirty;							// flag for updating ui (don't worry about this)

};

class SpringParticleSystem : public ParticleSystem {
public:
	SpringParticleSystem();
	virtual void drawParticles(float t);
	virtual void computeForcesAndUpdateParticles(float t);
	virtual void clearBaked();
	virtual double getRestLength() { return restLength; }
	virtual double getk() { return k; }
private:
	double restLength = 0.1;
	double k = 1;
};

class ClothParticleSystem : public SpringParticleSystem {
public:
	ClothParticleSystem();
	virtual void drawParticles(float t);
	virtual void computeForcesAndUpdateParticles(float t);
	virtual void clearBaked();
	virtual double* getL() { return L; }
	virtual double* getK() { return k; }
private:
	double L[3] = { 0.6,sqrt(2)*0.6,1.2 };
	double k[3] = { 10,5,1 };
};

class HairParticleSystem : public SpringParticleSystem {
public:
	HairParticleSystem();
	virtual void drawParticles(float t);
	virtual void computeForcesAndUpdateParticles(float t);
	virtual void clearBaked();
	virtual double* getL() { return L; }
	virtual double* getK() { return k; }
private:
	double L[2] = { 0.06,0.15 };
	double k[2] = { 25,25 };
};

class FlockParticleSystem : public ParticleSystem {
public:
	FlockParticleSystem();
	virtual void drawParticles(float t);
	virtual void computeForcesAndUpdateParticles(float t);
	virtual void clearBaked();
private:
	double r0 = 2, r1 = 4, r2 = 0.3;
	double localR = 2;
	Vec3d targetx;
};

#endif	// __PARTICLE_SYSTEM_H__
