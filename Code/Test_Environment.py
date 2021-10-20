
import numpy as np
from RK_Driver import rk_step
from Parameters_and_Constants import pi
import matplotlib.pyplot as plt

# Make a 2D plot of the results for the angular velocity
# Inputs:
# - Time array (time[ N ]) [sec]
# - Angular rate array (om_arr[ N ][ 3 ]) [rad/s]
# - fig-name:
# -- "0" if no figure is to be saved
# -- string which is the fig name otherwise
def plot_2D( time , om_arr , fig_name ):

    # Plot the results
    fig, ax = plt.subplots()

    ax.plot( time , np.transpose( om_arr )[ 0 ] , linestyle = "solid" , label = r"\omega_x" )
    ax.plot( time , np.transpose( om_arr )[ 1 ] , linestyle = "solid" , label = r"\omega_y" )
    ax.plot( time , np.transpose( om_arr )[ 2 ] , linestyle = "solid" , label = r"\omega_z" )

    plt.xlabel( "Time (s)" )
    plt.ylabel( "Angular Rate [rad/s]" )
    plt.title( "Intermediate Axis Plot" )

    plt.grid()

    if fig_name != "0":
        fig.savefig( fig_name + ".png" , format = "png" )

    plt.show( )


# Make a 3D animation of the results for the angular velocity
# Inputs:
# - Time array (time[ N ]) [sec]
# - Angular rate array (om_arr[ N ][ 3 ]) [rad/s]
def animate_3D( time , om_arr ):

    ax = plt.figure( ).add_subplot( projection = "3d" )    
    
    for i in range( 0 , len( time ) ):
        plt.cla( )
        ax.quiver( 0 , 0 , 0 , om_arr[ i ][ 0 ] , om_arr[ i ][ 1 ] , om_arr[ i ][ 2 ] , normalize = True )

        ax.set_xlim3d( -1.2 , 1.2 )
        ax.set_ylim3d( -1.2 , 1.2 )
        ax.set_zlim3d( -1.2 , 1.2 )

        plt.pause( 0.1 )

# To include 3D motion visualization of the 3-axis system -> body frame

if __name__ == "__main__":

    # Initial body attitude with respect to inertial frame [-]
    q_init = [ 1.0 , 0.0 , 0.0 , 0.0 ]

    # Initial body angular velocity in body-frame [rad/s]
    om_init = [ pi/10.0 , pi/2.0 , pi/10.0 ]

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
    animate_3D( time , om_arr )
