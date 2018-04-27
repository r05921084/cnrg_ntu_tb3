#!/usr/bin/env python
import numpy as np
import time
import threading
import nengo
from delay_line import DelayLine

import rospy
from std_msgs.msg import String
from std_msgs.msg import Header
# from binaural_microphone.msg import BinauralAudio
from ipem_module.msg import AuditoryNerveImage


NODE_NAME = 'nengo_mso_model'
PUB_TOPIC_NAME = 'mso_stream'
SUB_TOPIC_NAME = '/ipem_module/apm_stream'
SAMPLE_RATE = 11025
CHUNK_SIZE = 1024
N_SUBCHANNELS = 40

def aid(x):
    # This function returns the memory
    # block address of an array.
    return x.__array_interface__['data'][0]


def run_MSO_model():

    dl_L = DelayLine((N_SUBCHANNELS,), SAMPLE_RATE, initial_value=0.05)
    dl_R = DelayLine((N_SUBCHANNELS,), SAMPLE_RATE, initial_value=0.05)

    mso_pub = rospy.Publisher(PUB_TOPIC_NAME, AuditoryNerveImage, queue_size=1)

    reshape_index = np.arange(1024)[:,np.newaxis] * 40 + np.arange(40)
    ani_L = np.zeros([N_SUBCHANNELS * CHUNK_SIZE])
    ani_R = np.zeros([N_SUBCHANNELS * CHUNK_SIZE])
    event = threading.Event()

    def ani_cb(data):
        
        if data.chunk_size != CHUNK_SIZE or data.n_subchannels != N_SUBCHANNELS or data.sample_rate != SAMPLE_RATE:            
            rospy.logwarn('NOT IMPLEMENT YET: variable CHUNK_SIZE, N_SUBCHANNELS and SAMPLE_RATE not supported!')
            # CHUNK_SIZE = data.chunk_size
            # N_SUBCHANNELS = data.n_subchannels
            # SAMPLE_RATE = data.sample_rate
            # setup_objects()
            return
        # t1 = time.time()
        try:
            # ani_L[:] = np.asarray(data.left_channel).reshape([data.chunk_size, data.n_subchannels])
            # ani_R[:] = np.asarray(data.right_channel).reshape([data.chunk_size, data.n_subchannels])
            ani_L[:] = data.left_channel
            ani_R[:] = data.right_channel
        except ValueError:
            rospy.logwarn('shape mismatch: %d -> %d %d' % (len(data.left_channel), data.chunk_size, data.n_subchannels))
            return
        
        dl_L.update(ani_L[reshape_index])
        dl_R.update(ani_R[reshape_index])
        event.set()

        # print time.time() - t1

    def build_model():

        synapse1 = None
        synapse2 = 0
        synapse3 = 0
        synapse4 = None

        radius_1 = 1
        radius_2 = 2

        lifrate_model = nengo.LIFRate(tau_rc=0.002, tau_ref=0.0002)
        max_r = nengo.dists.Uniform(1000, 2000)

        delay_values = [0., 9.e-5, 26e-5, 41e-5, 56e-5, 73e-5, 82e-5]

        def MSO_ensemble_array(radius):
            return nengo.networks.EnsembleArray(n_neurons=32, n_ensembles=N_SUBCHANNELS, ens_dimensions=1,
                                                radius=radius, neuron_type=lifrate_model, max_rates=max_r, seed=None)

        with nengo.Network(label="MSO_Jeffress_Model_Multi_Channel") as network:
            # for i in range(len(delay_values)):
                
            input_node_L = nengo.Node(dl_L.delayed_view(0.))
            delay_node_L_A = nengo.Node(dl_L.delayed_view(9.e-5))
            delay_node_L_B = nengo.Node(dl_L.delayed_view(26e-5))
            delay_node_L_C = nengo.Node(dl_L.delayed_view(41e-5))
            delay_node_L_D = nengo.Node(dl_L.delayed_view(56e-5))
            delay_node_L_E = nengo.Node(dl_L.delayed_view(73e-5))
            delay_node_L_F = nengo.Node(dl_L.delayed_view(82e-5))

            input_node_R = nengo.Node(dl_R.delayed_view(0.))
            delay_node_R_A = nengo.Node(dl_R.delayed_view(9.e-5))
            delay_node_R_B = nengo.Node(dl_R.delayed_view(26e-5))
            delay_node_R_C = nengo.Node(dl_R.delayed_view(41e-5))
            delay_node_R_D = nengo.Node(dl_R.delayed_view(56e-5))
            delay_node_R_E = nengo.Node(dl_R.delayed_view(73e-5))
            delay_node_R_F = nengo.Node(dl_R.delayed_view(82e-5))

            # creating Ensembles
            ens_arr_L_1 = MSO_ensemble_array(radius_1)
            ens_arr_L_2 = MSO_ensemble_array(radius_1)
            ens_arr_L_3 = MSO_ensemble_array(radius_1)
            ens_arr_L_4 = MSO_ensemble_array(radius_1)
            ens_arr_L_5 = MSO_ensemble_array(radius_1)
            ens_arr_L_6 = MSO_ensemble_array(radius_1)
            ens_arr_L_7 = MSO_ensemble_array(radius_1)

            ens_arr_R_1 = MSO_ensemble_array(radius_1)
            ens_arr_R_2 = MSO_ensemble_array(radius_1)
            ens_arr_R_3 = MSO_ensemble_array(radius_1)
            ens_arr_R_4 = MSO_ensemble_array(radius_1)
            ens_arr_R_5 = MSO_ensemble_array(radius_1)
            ens_arr_R_6 = MSO_ensemble_array(radius_1)
            ens_arr_R_7 = MSO_ensemble_array(radius_1)

            ens_arr_add_1 = MSO_ensemble_array(radius_2)
            ens_arr_add_2 = MSO_ensemble_array(radius_2)
            ens_arr_add_3 = MSO_ensemble_array(radius_2)
            ens_arr_add_4 = MSO_ensemble_array(radius_2)
            ens_arr_add_5 = MSO_ensemble_array(radius_2)
            ens_arr_add_6 = MSO_ensemble_array(radius_2)
            ens_arr_add_7 = MSO_ensemble_array(radius_2)

            # connecting Nodes to Ensembles
            nengo.Connection(input_node_L,   ens_arr_L_1.input, synapse=synapse2)
            nengo.Connection(delay_node_L_A, ens_arr_L_2.input, synapse=synapse2)
            nengo.Connection(delay_node_L_B, ens_arr_L_3.input, synapse=synapse2)
            nengo.Connection(delay_node_L_C, ens_arr_L_4.input, synapse=synapse2)
            nengo.Connection(delay_node_L_D, ens_arr_L_5.input, synapse=synapse2)
            nengo.Connection(delay_node_L_E, ens_arr_L_6.input, synapse=synapse2)
            nengo.Connection(delay_node_L_F, ens_arr_L_7.input, synapse=synapse2)

            nengo.Connection(delay_node_R_F, ens_arr_R_7.input, synapse=synapse2)
            nengo.Connection(delay_node_R_E, ens_arr_R_6.input, synapse=synapse2)
            nengo.Connection(delay_node_R_D, ens_arr_R_5.input, synapse=synapse2)
            nengo.Connection(delay_node_R_C, ens_arr_R_4.input, synapse=synapse2)
            nengo.Connection(delay_node_R_B, ens_arr_R_3.input, synapse=synapse2)
            nengo.Connection(delay_node_R_A, ens_arr_R_2.input, synapse=synapse2)
            nengo.Connection(input_node_R,   ens_arr_R_1.input, synapse=synapse2)

            nengo.Connection(ens_arr_L_1.output, ens_arr_add_1.input, synapse=synapse3)
            nengo.Connection(ens_arr_L_2.output, ens_arr_add_2.input, synapse=synapse3)
            nengo.Connection(ens_arr_L_3.output, ens_arr_add_3.input, synapse=synapse3)
            nengo.Connection(ens_arr_L_4.output, ens_arr_add_4.input, synapse=synapse3)
            nengo.Connection(ens_arr_L_5.output, ens_arr_add_5.input, synapse=synapse3)
            nengo.Connection(ens_arr_L_6.output, ens_arr_add_6.input, synapse=synapse3)
            nengo.Connection(ens_arr_L_7.output, ens_arr_add_7.input, synapse=synapse3)

            nengo.Connection(ens_arr_R_7.output, ens_arr_add_1.input, synapse=synapse3)
            nengo.Connection(ens_arr_R_6.output, ens_arr_add_2.input, synapse=synapse3)
            nengo.Connection(ens_arr_R_5.output, ens_arr_add_3.input, synapse=synapse3)
            nengo.Connection(ens_arr_R_4.output, ens_arr_add_4.input, synapse=synapse3)
            nengo.Connection(ens_arr_R_3.output, ens_arr_add_5.input, synapse=synapse3)
            nengo.Connection(ens_arr_R_2.output, ens_arr_add_6.input, synapse=synapse3)
            nengo.Connection(ens_arr_R_1.output, ens_arr_add_7.input, synapse=synapse3)

            # Add Probes
            global input_node_L_probe, delay_node_L_A_probe, delay_node_L_B_probe, delay_node_L_C_probe,\
            delay_node_L_D_probe, delay_node_L_E_probe, delay_node_L_F_probe
            global input_node_R_probe, delay_node_R_A_probe, delay_node_R_B_probe, delay_node_R_C_probe,\
            delay_node_R_D_probe, delay_node_R_E_probe, delay_node_R_F_probe
            global ens_arr_add_1_probe, ens_arr_add_2_probe, ens_arr_add_3_probe, ens_arr_add_4_probe,\
            ens_arr_add_5_probe, ens_arr_add_6_probe, ens_arr_add_7_probe

            input_node_L_probe = nengo.Probe(input_node_L)
            delay_node_L_A_probe = nengo.Probe(delay_node_L_A)
            delay_node_L_B_probe = nengo.Probe(delay_node_L_B)
            delay_node_L_C_probe = nengo.Probe(delay_node_L_C)
            delay_node_L_D_probe = nengo.Probe(delay_node_L_D)
            delay_node_L_E_probe = nengo.Probe(delay_node_L_E)
            delay_node_L_F_probe = nengo.Probe(delay_node_L_F)

            input_node_R_probe = nengo.Probe(input_node_R)
            delay_node_R_A_probe = nengo.Probe(delay_node_R_A)
            delay_node_R_B_probe = nengo.Probe(delay_node_R_B)
            delay_node_R_C_probe = nengo.Probe(delay_node_R_C)
            delay_node_R_D_probe = nengo.Probe(delay_node_R_D)
            delay_node_R_E_probe = nengo.Probe(delay_node_R_E)
            delay_node_R_F_probe = nengo.Probe(delay_node_R_F)

            ens_arr_add_1_probe = nengo.Probe(ens_arr_add_1.output, synapse=synapse4)
            ens_arr_add_2_probe = nengo.Probe(ens_arr_add_2.output, synapse=synapse4)
            ens_arr_add_3_probe = nengo.Probe(ens_arr_add_3.output, synapse=synapse4)
            ens_arr_add_4_probe = nengo.Probe(ens_arr_add_4.output, synapse=synapse4)
            ens_arr_add_5_probe = nengo.Probe(ens_arr_add_5.output, synapse=synapse4)
            ens_arr_add_6_probe = nengo.Probe(ens_arr_add_6.output, synapse=synapse4)
            ens_arr_add_7_probe = nengo.Probe(ens_arr_add_7.output, synapse=synapse4)
            
            # import nengo_dl
            # return nengo_dl.Simulator(network, dt=1. / SAMPLE_RATE, unroll_simulation=64)
            import pyopencl
            import nengo_ocl
            import os
            os.environ[ 'PYOPENCL_CTX' ] = '0'
            ctx = pyopencl.create_some_context()
            sim = nengo_ocl.Simulator(network, context=ctx, dt=float(1. / SAMPLE_RATE), n_prealloc_probes=1024, if_python_code='warn')
            return sim


    sim = build_model()
    print sim.time, sim.n_steps
    
    rospy.Subscriber(SUB_TOPIC_NAME, AuditoryNerveImage, ani_cb)
    rospy.loginfo('"%s" starts subscribing to "%s".' % (NODE_NAME, SUB_TOPIC_NAME))

    # rospy.spin()

    while not rospy.is_shutdown():
        while event.wait(0.3):
            event.clear()
            steps_to_run = dl_L.current_step - sim.n_steps
            t2 = time.time()
            sim.run_steps(steps_to_run)
            print 'mainloop %dsteps: ' % steps_to_run, time.time() - t2
            steps_to_run = dl_L.current_step - sim.n_steps
            print 'next  %dsteps: ' % steps_to_run
            break
        break


if __name__ == '__main__':
    try:
        rospy.init_node(NODE_NAME, anonymous=False)
        run_MSO_model()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)