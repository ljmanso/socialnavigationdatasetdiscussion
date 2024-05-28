#ifndef ROBOCOMPPOSE3D_ICE
#define ROBOCOMPPOSE3D_ICE
module RoboCompPose3D
{
	sequence <double> Floats;
	sequence <Floats> Skeletons;

	interface Pose3D
	{
		Skeletons getskeletons();
	};
};

#endif
