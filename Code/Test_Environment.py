
import numpy as np
from RK_Driver import rk_step
from Parameters_and_Constants import pi
from Visualizations import plot_2D, animate_3D, animate_dreibein

if __name__ == "__main__":

    # Initial body attitude with respect to inertial frame [-]
    q_init = [ 1.0 , 0.0 , 0.0 , 0.0 ]

    # Initial body angular velocity in body-frame [rad/s]
    om_init = [ 0.0 , pi/3.0 , pi/20.0 ]

    npoints = 1000 # Number of points for integrator [-]
    time_tot = 120.0 # Simulation time [sec]

    time = np.linspace( 0 , time_tot , npoints ) # Create time array
    dt = time_tot/( npoints - 1.0 ) # Get the number of intervals for integration

    # Create arrays where to keep the attitude and angular rate data for plotting
    q_arr = np.zeros( ( npoints , 4 ) )
    om_arr = np.zeros( ( npoints , 3 ) )

    # Populate the arrays with the initial values
    q_arr[ 0 ] = q_init
    om_arr[ 0 ] = om_init

    # Propagate for each of the points
    for i in range( 0 , npoints - 1 ):
        [ q_arr[ i + 1 ] , om_arr[ i + 1 ] ] = rk_step( [ q_arr[ i ] , om_arr[ i ] ] , time[ i ] , dt )
    
    #plot_2D( time , om_arr , "2D_Plot" )
    #animate_3D( time , om_arr )
    animate_dreibein( time , q_arr )
