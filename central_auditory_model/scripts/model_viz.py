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

    synapse1 = None
    synapse2 = 0
    synapse3 = 0
    synapse4 = None

    radius_1 = 1
    radius_2 = 2

    dl_L = DelayLine((N_SUBCHANNELS,), SAMPLE_RATE, initial_value=0.05, max_delay=MAX_DELAY)
    dl_R = DelayLine((N_SUBCHANNELS,), SAMPLE_RATE, initial_value=0.05, max_delay=MAX_DELAY)

    lifrate_model = nengo.LIFRate(tau_rc=0.002, tau_ref=0.0002)
    max_r = nengo.dists.Uniform(1000, 2000)

    delay_values = [0., 9.e-5, 26e-5, 41e-5, 56e-5, 73e-5, 82e-5]
    # delay_values = [0., 9.e-5, 26e-5]

    def MSO_ensemble_array(radius, label='ens_arr'):
        return nengo.networks.EnsembleArray(n_neurons=32, n_ensembles=N_SUBCHANNELS, ens_dimensions=1, label=label,
                                            radius=radius, neuron_type=lifrate_model, max_rates=max_r, seed=None)

    model = nengo.Network(label="MSO_Jeffress_Model_Multi_Channel")
    with model:
        input_node_L = []
        input_node_R = []
        ens_arr_L = []
        ens_arr_R = []
        ens_arr_add = []

        for i in range(len(delay_values)):
            input_node_L.append(nengo.Node(dl_L.delayed_view(delay_values[i]), label='input_node_L[%d]' % i))
            input_node_R.append(nengo.Node(dl_R.delayed_view(delay_values[i]), label='input_node_R[%d]' % i))
            ens_arr_L.append(MSO_ensemble_array(radius_1, label='ens_arr_L[%d]' % i))
            ens_arr_R.append(MSO_ensemble_array(radius_1, label='ens_arr_R[%d]' % i))
            ens_arr_add.append(MSO_ensemble_array(radius_2, label='ens_arr_add[%d]' % i))

        for i in range(len(delay_values)):
            nengo.Connection(input_node_L[i],   ens_arr_L[i].input, synapse=synapse2)
            nengo.Connection(input_node_R[i],   ens_arr_R[i].input, synapse=synapse2)
            nengo.Connection(ens_arr_L[i].output, ens_arr_add[i].input, synapse=synapse3)
            nengo.Connection(ens_arr_R[-1-i].output, ens_arr_add[i].input, synapse=synapse3)
