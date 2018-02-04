/*
 * Observer.cpp
 *
 *  Created on: Feb 3, 2018
 *      Author: maxkwon
 */

#include <Observer.h>

namespace std {

std::vector<double > Y = {0};

Observer::Observer() {



}


void Observer::PredictObserver(){



}


void Observer::CorrectObsevrer(){



}

std::vector<double > Observer::GetXHat(){



}

void Observer::SetY(std::vector<double > y){

	Y = y;

}

}
