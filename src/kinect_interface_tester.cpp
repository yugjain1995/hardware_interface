#include <kinect_interface.h>


int main()
{
    int stat = 0;
    /// Get RGB frames from kinect interface
    /// frame = get_rgb_frames();
    stat = get_rgb_frames();
    /// Display RGB frames and frame rate

    return stat;
}