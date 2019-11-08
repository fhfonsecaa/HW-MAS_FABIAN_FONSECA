/**
* @file ContextCoordinator.cpp
*   This file implements the team coordination module
* @author Francesco Riccio, Emanuele Borzi
*/

#include "ContextCoordinator.h"

#include <unistd.h>
#include <iostream>
#include "Representations/SPQR-Libraries/ConfigFile/ConfigFile.h"


//#include <fstream> // ofstream


#define NORM(x, y) sqrt(x*x + y*y)
//#define UM_VIEW

MAKE_MODULE(ContextCoordinator, behaviorControl)

ContextCoordinator::ContextCoordinator(){
    SPQR::ConfigurationParameters();
}


void ContextCoordinator::update(Role& role){ 
    role.role = Role::RoleType::taker;
    role.role = Role::RoleType::kicker;
    return;
}
