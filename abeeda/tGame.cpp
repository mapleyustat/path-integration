/*
 * tGame.cpp
 *
 * This file is part of the aBeeDa Swarm Evolution project.
 *
 * Copyright 2012 Randal S. Olson, Arend Hintze.
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "tGame.h"
#include <math.h>
#include <float.h>
#include <stdlib.h>
#include <stdio.h>

#define cPI 3.14159265

// simulation-specific constants
#define pathfinderVisionRange 50.0 * 50.0
#define pathfinderVisionAngle 180.0 / 2.0
#define pathfinderSensors 4
#define totalStepsInSimulation 2000
#define gridX 256.0
#define gridY 256.0
#define contactDist 20.0 * 20.0
#define boundaryDist 250.0

// precalculated lookup tables for the game
double cosLookup[360];
double sinLookup[360];

tGame::tGame()
{
    // fill lookup tables
    for (int i = 0; i < 360; ++i)
    {
        cosLookup[i] = cos((double)i * (cPI / 180.0));
        sinLookup[i] = sin((double)i * (cPI / 180.0));
    }
}

tGame::~tGame() { }

// runs the simulation for the given agent(s)
string tGame::executeGame(tAgent* pathfinderAgent, FILE *data_file, bool report)
{
    string reportString = "";
    double pathfinderX = 0.0, pathfinderY = 0.0, pathfinderAngle = 0.0, pathfinderFitness = 0.0;
    bool pathfinderHasFood = false;
    double hiveX = 0.0, hiveY = 0.0;
    double foodX = 0.0, foodY = 0.0;
    double sunX = 0.0, sunY = -gridY;
    int scentDelay = 0;
    int totalFood = 0;
    vector<double> scentLocationX;
    vector<double> scentLocationY;
    
    pathfinderAgent->setupPhenotype();
    
    pathfinderX = 200.0;
    pathfinderY = 200.0;
    //pathfinderAngle = (int)(randDouble * 360.0);
    
    hiveX = 200.0;
    hiveY = 205.0;
    
    foodX = -200.0;//(2.0 * randDouble * gridX) - gridX;
    foodY = -200.0;//-randDouble * gridY;
    
    /*       BEGINNING OF SIMULATION LOOP       */
    
    int step = 0;
    
    for( ; step < totalStepsInSimulation; ++step)
    {
        
        /*       CREATE THE REPORT STRING FOR THE VIDEO       */
        if(report)
        {
            // report X, Y of starting point point
            char text[1000];
            sprintf(text,"%f,%f,%f,%d,%d,%d=", hiveX, hiveY, 0.0, 255, 255, 0);
            reportString.append(text);
            
            // report X, Y, angle of pathfinding agent
            char text1[1000];
            
            if (pathfinderHasFood)
            {
                sprintf(text1,"%f,%f,%f,%d,%d,%d=", pathfinderX, pathfinderY, pathfinderAngle, 30, 144, 255);
            }
            else
            {
                sprintf(text1,"%f,%f,%f,%d,%d,%d=", pathfinderX, pathfinderY, pathfinderAngle, 255, 0, 0);
            }
            reportString.append(text1);
            
            // report X, Y of food point
            char text2[1000];
            sprintf(text2,"%f,%f,%f,%d,%d,%d=", foodX, foodY, 0.0, 0, 255, 0);
            reportString.append(text2);
            
            for (int i = 0; i < scentLocationX.size(); ++i)
            {
                char text3[1000];
                sprintf(text3,"%f,%f,%f,%d,%d,%d=", scentLocationX[i], scentLocationY[i], 0.0, 255, 255, 255);
                reportString.append(text3);
            }
            
            reportString.append("N");
            
        }
        /*       END OF REPORT STRING CREATION       */
        
        
        /*       SAVE DATA FOR THE LOD FILE       */
        if(data_file != NULL)
        {
            
        }
        /*       END OF DATA GATHERING       */
        
        
        /*       UPDATE PATHFINDER       */
        
        // clear the pathfinder sensors
        for(int i = 0; i < (pathfinderSensors * 3) + 1; ++i)
        {
            pathfinderAgent->states[i] = 0;
        }
        
        // update the pathfinder sensors
        
        // food
        if(calcDistanceSquared(pathfinderX, pathfinderY, foodX, foodY) < pathfinderVisionRange)
        {
            double angle = calcAngle(pathfinderX, pathfinderY, pathfinderAngle, foodX, foodY);
            
            // here we have to map the angle into the sensor, angle in degrees
            if(fabs(angle) < pathfinderVisionAngle) // pathfinder has a limited vision field in front of it
            {
                pathfinderAgent->states[(int)(angle / (pathfinderVisionAngle / ((double)pathfinderSensors / 2.0)) + ((double)pathfinderSensors / 2.0))] = 1;
            }
        }
        
        // hive
        if(calcDistanceSquared(pathfinderX, pathfinderY, hiveX, hiveY) < pathfinderVisionRange)
        {
            double angle = calcAngle(pathfinderX, pathfinderY, pathfinderAngle, hiveX, hiveY);
            
            // here we have to map the angle into the sensor, angle in degrees
            if(fabs(angle) < pathfinderVisionAngle) // pathfinder has a limited vision field in front of it
            {
                pathfinderAgent->states[pathfinderSensors + (int)(angle / (pathfinderVisionAngle / ((double)pathfinderSensors / 2.0)) + ((double)pathfinderSensors / 2.0))] = 1;
            }
        }
        
        // scent
        for (int i = 0; i < scentLocationX.size(); ++i)
        {
            if(calcDistanceSquared(pathfinderX, pathfinderY, scentLocationX[i], scentLocationY[i]) < pathfinderVisionRange)
            {
                double angle = calcAngle(pathfinderX, pathfinderY, pathfinderAngle, scentLocationX[i], scentLocationY[i]);
                
                // here we have to map the angle into the sensor, angle in degrees
                if(fabs(angle) < pathfinderVisionAngle) // pathfinder has a limited vision field in front of it
                {
                    pathfinderAgent->states[(pathfinderSensors * 2) + (int)(angle / (pathfinderVisionAngle / ((double)pathfinderSensors / 2.0)) + ((double)pathfinderSensors / 2.0))] = 1;
                }
            }
        }
        
        // sun
        double angle = calcAngle(pathfinderX, pathfinderY, pathfinderAngle, sunX, sunY);
            
        // here we have to map the angle into the sensor, angle in degrees
        pathfinderAgent->states[(pathfinderSensors * 3) + (int)(angle / (180.0 / ((double)pathfinderSensors / 2.0)) + ((double)pathfinderSensors / 2.0))] = 1;
        
        // has food?
        if (pathfinderHasFood)
        {
            pathfinderAgent->states[(pathfinderSensors * 3) + 1] = 1;
        }
        else
        {
            pathfinderAgent->states[(pathfinderSensors * 3) + 1] = 0;
        }
        
        // activate the pathfinder's brain
        pathfinderAgent->updateStates();
        
        //                                      node 31                                              node 30
        int action = ((pathfinderAgent->states[(maxNodes - 1)] & 1) << 1) + (pathfinderAgent->states[(maxNodes - 2)] & 1);
        
        switch(action)
        {
                // move straight ahead
            case 0:
                pathfinderX += cosLookup[(int)pathfinderAngle] * 2.0;
                pathfinderY += sinLookup[(int)pathfinderAngle] * 2.0;
                break;
                
                // turn right
            case 1:
                pathfinderAngle += 6.0;
                
                while(pathfinderAngle >= 360.0)
                {
                    pathfinderAngle -= 360.0;
                }
                
                pathfinderX += cosLookup[(int)pathfinderAngle] * 2.0;
                pathfinderY += sinLookup[(int)pathfinderAngle] * 2.0;
                
                break;
                
                // turn left
            case 2:
                pathfinderAngle -= 6.0;
                
                while(pathfinderAngle < 0.0)
                {
                    pathfinderAngle += 360.0;
                }
                
                pathfinderX += cosLookup[(int)pathfinderAngle] * 2.0;
                pathfinderY += sinLookup[(int)pathfinderAngle] * 2.0;
                
                break;
                
                // mark location with scent
            case 3:
                if (scentDelay < 1)
                {
                    scentDelay = 25;
                    
                    scentLocationX.push_back(pathfinderX);
                    scentLocationY.push_back(pathfinderY);
                    
                    if (scentLocationX.size() > 25)
                    {
                        scentLocationX.erase(scentLocationX.begin());
                        scentLocationY.erase(scentLocationY.begin());
                    }
                }
                break;
                
            default:
                break;
        }
        
        // keep position within simulation boundary
        applyBoundary(pathfinderX);
        applyBoundary(pathfinderY);
        
        if (scentDelay > 0)
        {
            --scentDelay;
        }
        
        /*       END OF PATHFINDER UPDATE       */
        
        /*      STEP FITNESS EVALUATION         */
        
        if (!pathfinderHasFood)
        {
            pathfinderFitness += (1.0 / calcDistanceSquared(pathfinderX, pathfinderY, foodX, foodY)) * 500.0;
        }
        else
        {
            pathfinderFitness += (1.0 / calcDistanceSquared(pathfinderX, pathfinderY, hiveX, hiveY)) * 500.0 * 10.0;
        }
        
        /*      END STEP FITNESS EVALUATION         */
        
        /*      STEP STATE UPDATE               */
        
        if (!pathfinderHasFood && calcDistanceSquared(pathfinderX, pathfinderY, foodX, foodY) < contactDist)
        {
            pathfinderHasFood = true;
        }
        
        if (pathfinderHasFood && calcDistanceSquared(pathfinderX, pathfinderY, hiveX, hiveY) < contactDist)
        {
            pathfinderHasFood = false;
            pathfinderFitness += (double)(totalStepsInSimulation - step);
            ++totalFood;
        }
        
        /*      END STEP STATE UPDATE           */
        
    }
    /*       END OF SIMULATION LOOP       */
    
    // compute overall fitness
    pathfinderAgent->fitness = max(pathfinderFitness, 0.0);
    
    // output to data file, if provided
    if (data_file != NULL)
    {
        fprintf(data_file, "%d,%f,%d\n",
                pathfinderAgent->born,              // update born (prey)
                pathfinderAgent->fitness,           // pathfinder fitness
                totalFood                           // total food brought back to nest
                );
    }
    
    return reportString;
}


// calculates the distance^2 between two points
double tGame::calcDistanceSquared(double fromX, double fromY, double toX, double toY)
{
    double diffX = fromX - toX;
    double diffY = fromY - toY;
    
    return ( diffX * diffX ) + ( diffY * diffY );
}

// calculates the angle between two agents
double tGame::calcAngle(double fromX, double fromY, double fromAngle, double toX, double toY)
{
    double Ux = 0.0, Uy = 0.0, Vx = 0.0, Vy = 0.0;
    
    Ux = (toX - fromX);
    Uy = (toY - fromY);
    
    Vx = cosLookup[(int)fromAngle];
    Vy = sinLookup[(int)fromAngle];
    
    int firstTerm = (int)((Ux * Vy) - (Uy * Vx));
    int secondTerm = (int)((Ux * Vx) + (Uy * Vy));
    
    return atan2(firstTerm, secondTerm) * 180.0 / cPI;
    //return atan2Lookup[firstTerm + 400][secondTerm + 400];
}

// maintains a position within a preset boundary
void tGame::applyBoundary(double& positionVal)
{
    double val = positionVal;
    
    if (fabs(val) > boundaryDist)
    {
        if (val < 0)
        {
            val = -1.0 * boundaryDist;
        }
        else
        {
            val = boundaryDist;
        }
    }
    
    positionVal = val;
}

// sums a vector of values
double tGame::sum(vector<double> values)
{
    double sum = 0.0;
    
    for (vector<double>::iterator i = values.begin(); i != values.end(); ++i)
    {
        sum += *i;
    }
    
    return sum;
}

// averages a vector of values
double tGame::average(vector<double> values)
{
    return sum(values) / (double)values.size();
}

// computes the variance of a vector of values
double tGame::variance(vector<double> values)
{
    double sumSqDist = 0.0;
    double mean = average(values);
    
    for (vector<double>::iterator i = values.begin(); i != values.end(); ++i)
    {
        sumSqDist += pow( *i - mean, 2.0 );
    }
    
    return sumSqDist /= (double)values.size();
}

double tGame::mutualInformation(vector<int> A,vector<int>B)
{
	set<int> nrA,nrB;
	set<int>::iterator aI,bI;
	map<int,map<int,double> > pXY;
	map<int,double> pX,pY;
	int i;
	double c=1.0/(double)A.size();
	double I=0.0;
	for(i=0;i<A.size();i++){
		nrA.insert(A[i]);
		nrB.insert(B[i]);
		pX[A[i]]=0.0;
		pY[B[i]]=0.0;
	}
	for(aI=nrA.begin();aI!=nrA.end();aI++)
		for(bI=nrB.begin();bI!=nrB.end();bI++){
			pXY[*aI][*bI]=0.0;
		}
	for(i=0;i<A.size();i++){
		pXY[A[i]][B[i]]+=c;
		pX[A[i]]+=c;
		pY[B[i]]+=c;
	}
	for(aI=nrA.begin();aI!=nrA.end();aI++)
		for(bI=nrB.begin();bI!=nrB.end();bI++)
			if((pX[*aI]!=0.0)&&(pY[*bI]!=0.0)&&(pXY[*aI][*bI]!=0.0))
				I+=pXY[*aI][*bI]*log2(pXY[*aI][*bI]/(pX[*aI]*pY[*bI]));
	return I;
	
}

double tGame::entropy(vector<int> list){
	map<int, double> p;
	map<int,double>::iterator pI;
	int i;
	double H=0.0;
	double c=1.0/(double)list.size();
	for(i=0;i<list.size();i++)
		p[list[i]]+=c;
	for (pI=p.begin();pI!=p.end();pI++) {
        H+=p[pI->first]*log2(p[pI->first]);	
	}
	return -1.0*H;
}

double tGame::ei(vector<int> A,vector<int> B,int theMask){
	set<int> nrA,nrB;
	set<int>::iterator aI,bI;
	map<int,map<int,double> > pXY;
	map<int,double> pX,pY;
	int i;
	double c=1.0/(double)A.size();
	double I=0.0;
	for(i=0;i<A.size();i++){
		nrA.insert(A[i]&theMask);
		nrB.insert(B[i]&theMask);
		pX[A[i]&theMask]=0.0;
		pY[B[i]&theMask]=0.0;
	}
	for(aI=nrA.begin();aI!=nrA.end();aI++)
		for(bI=nrB.begin();bI!=nrB.end();bI++){
			pXY[*aI][*bI]=0.0;
		}
	for(i=0;i<A.size();i++){
		pXY[A[i]&theMask][B[i]&theMask]+=c;
		pX[A[i]&theMask]+=c;
		pY[B[i]&theMask]+=c;
	}
	for(aI=nrA.begin();aI!=nrA.end();aI++)
		for(bI=nrB.begin();bI!=nrB.end();bI++)
			if((pX[*aI]!=0.0)&&(pY[*bI]!=0.0)&&(pXY[*aI][*bI]!=0.0))
				I+=pXY[*aI][*bI]*log2(pXY[*aI][*bI]/(pY[*bI]));
	return -I;
}

double tGame::computeAtomicPhi(vector<int>A,int states){
	int i;
	double P,EIsystem;
	vector<int> T0,T1;
	T0=A;
	T1=A;
	T0.erase(T0.begin()+T0.size()-1);
	T1.erase(T1.begin());
	EIsystem=ei(T0,T1,(1<<states)-1);
	P=0.0;
	for(i=0;i<states;i++){
		double EIP=ei(T0,T1,1<<i);
        //		cout<<EIP<<endl;
		P+=EIP;
	}
    //	cout<<-EIsystem+P<<" "<<EIsystem<<" "<<P<<" "<<T0.size()<<" "<<T1.size()<<endl;
	return -EIsystem+P;
}

double tGame::computeR(vector<vector<int> > table,int howFarBack){
	double Iwh,Iws,Ish,Hh,Hs,Hw,Hhws,delta,R;
	int i;
	for(i=0;i<howFarBack;i++){
		table[0].erase(table[0].begin());
		table[1].erase(table[1].begin());
		table[2].erase(table[2].begin()+(table[2].size()-1));
	}
	table[4].clear();
	for(i=0;i<table[0].size();i++){
		table[4].push_back((table[0][i]<<14)+(table[1][i]<<10)+table[2][i]);
	}
	Iwh=mutualInformation(table[0],table[2]);
    Iws=mutualInformation(table[0],table[1]);
    Ish=mutualInformation(table[1],table[2]);
    Hh=entropy(table[2]);
    Hs=entropy(table[1]);
    Hw=entropy(table[0]);
    Hhws=entropy(table[4]);
    delta=Hhws+Iwh+Iws+Ish-Hh-Hs-Hw;
    R=Iwh-delta;
  	return R;
}

double tGame::computeOldR(vector<vector<int> > table){
	double Ia,Ib;
	Ia=mutualInformation(table[0], table[2]);
	Ib=mutualInformation(table[1], table[2]);
	return Ib-Ia;
}

double tGame::predictiveI(vector<int>A){
	vector<int> S,I;
	S.clear(); I.clear();
	for(int i=0;i<A.size();i++){
		S.push_back((A[i]>>12)&15);
		I.push_back(A[i]&3);
	}
	return mutualInformation(S, I);
}

double tGame::nonPredictiveI(vector<int>A){
	vector<int> S,I;
	S.clear(); I.clear();
	for(int i=0;i<A.size();i++){
		S.push_back((A[i]>>12)&15);
		I.push_back(A[i]&3);
	}
	return entropy(I)-mutualInformation(S, I);
}

double tGame::predictNextInput(vector<int>A){
	vector<int> S,I;
	S.clear(); I.clear();
	for(int i=0;i<A.size();i++){
		S.push_back((A[i]>>12)&15);
		I.push_back(A[i]&3);
	}
	S.erase(S.begin());
	I.erase(I.begin()+I.size()-1);
	return mutualInformation(S, I);
}

void tGame::loadExperiment(char *filename){
    theExperiment.loadExperiment(filename);
}

//** tOctuplet implementation
void tOctuplet::loadOctuplet(FILE *f){
    int i,IN;
    data.clear();
    data.resize(8);
    for(i=0;i<8;i++){
        fscanf(f,"  %i",&IN);
        data[i]=IN;
    }
}

//** tEperiment class implementations
void tExperiment::loadExperiment(char *filename){
    FILE *f=fopen(filename,"r+t");
    int i,j,k;
    fscanf(f,"%i:",&j);
    dropSequences.resize(j);
    for(i=0;i<dropSequences.size();i++)
        dropSequences[i].loadOctuplet(f);
    fscanf(f,"%i:",&j);
    sizeSequences.resize(j);
    for(i=0;i<sizeSequences.size();i++)
        sizeSequences[i].loadOctuplet(f);
    fscanf(f,"%i:",&j);
    selfSequences.resize(j);
    for(i=0;i<selfSequences.size();i++)
        selfSequences[i].loadOctuplet(f);
    shouldHit.resize(drops());
    for(i=0;i<shouldHit.size();i++){
        shouldHit[i].resize(sizes());
        for(j=0;j<shouldHit[i].size();j++){
            shouldHit[i][j].resize(selves());
            for(k=0;k<shouldHit[i][j].size();k++){
                int l;
                fscanf(f,"%i\n",&l);
                if(l==1)
                    shouldHit[i][j][k]=true;
                else
                    shouldHit[i][j][k]=false;
            }
        }
    }
    fclose(f);
}

void tExperiment::showExperimentProtokoll(void){
    int i,j,k;
    printf("drop directions: %i\n",drops());
    for(i=0;i<drops();i++){
        printf("%i:",i);
        for(j=0;j<8;j++)
            printf("    %i",dropSequences[i].data[j]);
        printf("\n");
    }
    printf("drop sizes: %i\n",sizes());
    for(i=0;i<sizes();i++){
        printf("%i:",i);
        for(j=0;j<8;j++)
            printf("    %i",sizeSequences[i].data[j]);
        printf("\n");
    }
    printf("self sizes: %i\n",selves());
    for(i=0;i<selves();i++){
        printf("%i:",i);
        for(j=0;j<8;j++)
            printf("    %i",selfSequences[i].data[j]);
        printf("\n");
    }
    printf("should hit\n%i means true\nD  B   S   catch\n",(int)true);
    for(i=0;i<shouldHit.size();i++)
        for(j=0;j<shouldHit[i].size();j++)
            for(k=0;k<shouldHit[i][j].size();k++)
                printf("%i  %i  %i  %i\n",i,j,k,(int)shouldHit[i][j][k]);
}

int tExperiment::drops(void){
    return (int) dropSequences.size();
}

int tExperiment::sizes(void){
    return (int) sizeSequences.size();
}

int tExperiment::selves(void){
    return (int) selfSequences.size();
    
}
