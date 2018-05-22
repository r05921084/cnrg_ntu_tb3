import os
os.environ['PYOPENCL_CTX'] = '0'

import nengo
import nengo_gui

from delay_line import DelayLine

if __name__ == '__main__':
    nengo_gui.GUI(__file__).start()
else:
    N_SUBCHANNELS = 40
    SAMPLE_RATE = 11025
    MAX_DELAY = 10.

    synapse1 = 0
    synapse2 = 0
    synapse3 = 0

    radius_1 = 1
    radius_2 = 2

    lifrate_model = nengo.LIFRate(tau_rc=0.002, tau_ref=0.0002)
    max_r = nengo.dists.Uniform(1000, 2000)

    def raw_ensemble_array(dim, radius, label='ens_arr', n_neurons=32):
        ens_arr = []
        for i in range(dim):
            ens_arr.append(nengo.Ensemble(n_neurons, 1, radius=radius, neuron_type=lifrate_model, max_rates=max_r, label='%s_%d' % (label, i)))

        return ens_arr

    def cb(t, x):
        print x.shape

    with nengo.Network(label="MSO_Jeffress_Model_Multi_Channel") as model:
        input_node_L = nengo.Node(size_in=0, size_out=N_SUBCHANNELS, label='input_node_L')
        input_node_R = nengo.Node([0]*N_SUBCHANNELS, label='input_node_R')

        ens_arr_L = raw_ensemble_array(N_SUBCHANNELS, radius_1, 'ens_arr_L')
        ens_arr_R = raw_ensemble_array(N_SUBCHANNELS, radius_1, 'ens_arr_R')
        ens_arr_add = raw_ensemble_array(N_SUBCHANNELS, radius_2, 'ens_arr_add')

        output_node = nengo.Node(size_in=N_SUBCHANNELS, size_out=0, label='output_node')

        for i in range(N_SUBCHANNELS):
            nengo.Connection(input_node_L[i], ens_arr_L[i], synapse=synapse1)
            nengo.Connection(input_node_R[i], ens_arr_R[i], synapse=synapse1)
            nengo.Connection(ens_arr_L[i], ens_arr_add[i], synapse=synapse2)
            nengo.Connection(ens_arr_R[i], ens_arr_add[i], synapse=synapse2)
            nengo.Connection(ens_arr_add[i], output_node[i], synapse=synapse3)
