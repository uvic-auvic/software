//
// Created by avlec on 22/07/19.
//

#ifndef POLARIS_STATUSSYSTEM_HPP
#define POLARIS_STATUSSYSTEM_HPP

/*
 * \brief This class contains all numeric information about the submarine.
 *
 */
struct StatusSystem {
	/* Depth and Pressure Information */
	double pressure_external;
	double pressure_internal;
	double depth;

	/* Velocity Information */
	double vel_x;
	double vel_y;
	double vel_z;

	/* Angular Information */
	double pitch;
	double roll;
	double yaw;
};

#endif //POLARIS_STATUSSYSTEM_HPP
