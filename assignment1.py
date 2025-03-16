import numpy as np
from sim.sim1d import sim_run

# Simulator options.
options = {}
options['FIG_SIZE'] = [8,8]
options['FULL_RECALCULATE'] = False

class ModelPredictiveControl:
    def __init__(self):
        self.horizon = 20
        self.dt = 0.2

        # Reference or set point the controller will achieve.
        self.reference = [50, 0, 0]

    
    def plant_model(self, prev_state, dt, pedal, steering):
        x_t = prev_state[0]
        v_t = prev_state[3] # m/s
        x_t_1 = x_t + v_t*dt;
        v_t_1 = v_t + pedal*dt - v_t/25;
        
        return [x_t_1, 0, 0, v_t_1]

    def cost_function(self,u, *args):
        state = args[0]
        ref = args[1]
        cost = 0;
        for i in range(0,self.horizon):
            state = self.plant_model(state,self.dt,u[2*i],u[2*i + 1])
            
            #position cost
            cost += (state[0] - ref[0])**2;
           
            #velocity cost- velocity limit 10 km/hr or 2.77 m/s
            vel_curr = state[3];
            vel_lim = 2.77;
            
            if vel_curr >= (vel_lim):
                cost+=1000
            
            
        return cost

sim_run(options, ModelPredictiveControl)
