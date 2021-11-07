import numpy as np
from RK_Driver import rk_step
from Parameters_and_Constants import pi
from Visualizations import plot_2D_comparison
from Parameters_and_Constants import moi

# Generate perturbative solution arrays for Z-rotation and X-perturbation
# Inputs:
# - number of points (npoints)
# - total time (time_tot) in [sec] assuming that we start from t=0
# - primary angular rate along Z main_Om [rad/s] (constant solution)
# - perturbative angular rate along X dom [rad/s] (in the intial moment)
# Outputs:
# - time array with npoints between 0 and time_tot t_pert[ npoints ]
# - angular rate array with the X, Y and Z components of the perturbative solution om_pert[ npoints ][ 3 ]
# NOTE: This solution will only work for diagonal moment of inertia - transform the coordinates if it's not!
def pert_sol_arrays( npoints , time_tot , main_Om , dom ):

    # Kappa computation
    kap = np.sqrt( abs( ( moi[ 1 ][ 1 ] - moi[ 2 ][ 2 ] )*( moi[ 2 ][ 2 ] - moi[ 0 ][ 0 ] )*main_Om*main_Om/( moi[ 0 ][ 0 ]*moi[ 1 ][ 1 ] ) ) )
    thet = - ( moi[ 1 ][ 1 ] - moi[ 2 ][ 2 ] )*main_Om/moi[ 0 ][ 0 ]
    t_pert = np.linspace( 0 , time_tot , npoints )
    om_pert = np.zeros( ( npoints , 3 ) )
    
    for i in range( 0 , npoints ):
        om_pert[ i ] = [ dom*np.cos( kap*t_pert[ i ] ) , dom*np.sin( kap*t_pert[ i ] ) , main_Om ]

    return t_pert , om_pert

if __name__ == "__main__":

    # Initial body attitude with respect to inertial frame [-]
    q_init = [ 1.0 , 0.0 , 0.0 , 0.0 ]

    # Initial body angular velocity in body-frame [rad/s]
    om_init = [ pi/( 6.0 ) , 0.0 , pi/3.0 ]
    rat_om = om_init[ 0 ]/om_init[ 2 ] # scale of \delta \omega compared to \Omega (ratio)

    npoints = 1000 # Number of points for integrator [-]
    time_tot = 24.0 # Simulation time [sec]

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
    
    main_Om = om_init[ 2 ] # Z-axis (\Omega) around which the body is rotating [rad/s]
    dom = om_init[ 0 ] # Perturbation (\delta \omega) along X axis [rad/s]
    t_pert, om_pert = pert_sol_arrays( npoints , time_tot , main_Om , dom )

    plot_2D_comparison( time , t_pert , om_arr , om_pert , "Num_vs_Perturbed_Comparison_" + str( "{0:.3f}".format( rat_om ) ) + "scale" )