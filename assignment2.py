import numpy as np
from sim.sim2d import sim_run

# Simulator options.
options = {}
options['FIG_SIZE'] = [8,8]
options['OBSTACLES'] = False

class ModelPredictiveControl:
    def __init__(self):
        self.horizon = 20;
        self.dt = 0.2

        # Reference or set point the controller will achieve.
        self.reference1 = [10, 10, 0];
        self.reference2 = [10, 2, 3*3.14/2]

    def plant_model(self,prev_state, dt, pedal, steering):
        #state = [x,y,psi,v]
        x_t = prev_state[0]
        y_t = prev_state[1]
        psi_t = prev_state[2]
        v_t = prev_state[3]
        
        x_t_1 = x_t+ (v_t*np.cos(psi_t)*dt);
        y_t_1 = y_t+ (v_t*np.sin(psi_t)*dt);
        psi_t_1 = psi_t + (v_t/2.5)*np.tan(steering)*dt;
        v_t_1 = v_t+ pedal*dt - (v_t/25);

        return [x_t_1, y_t_1, psi_t_1, v_t_1]

    def cost_function(self,u, *args):  #u=[pedal1,steering1, pedal2,steering2, ....]
        state = args[0]
        ref = args[1]
        x_g = ref[0];
        y_g = ref[1];
        psi_g = ref[2];
        cost=0;
        for i in range(0,self.horizon):
            v_prev = state[3];
            state = self.plant_model(state,self.dt,u[2*i],u[2*i+1])
            x_n = state[0];
            y_n = state[1];
            psi_n = state[2];
            v_n = state[3];
            
            #goal distance cost
            cost += abs(x_g-x_n) + abs(y_g-y_n);
            #yaw angle cost
            cost += abs(psi_g-psi_n)**2;
            
           
            #acceleration cost
            a_n = (v_n - v_prev)/self.dt;
            
            if a_n >1.5:
                cost +=200;
            #cost += 1000*((0.8)*c1 + (0.2)*c2)/(c1+c2);
            
        return cost

sim_run(options, ModelPredictiveControl)
