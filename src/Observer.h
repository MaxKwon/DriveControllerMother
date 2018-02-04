/*
 * Observer.h
 *
 *  Created on: Feb 3, 2018
 *      Author: maxkwon
 */
#include <vector>

#ifndef SRC_OBSERVER_H_
#define SRC_OBSERVER_H_

namespace std {

class Observer {
public:

	Observer();

	void CorrectObsevrer();
	void PredictObserver();

	void SetY(std::vector<double > y);

	std::vector<double> GetXHat();

};

} /* namespace std */

#endif /* SRC_OBSERVER_H_ */
