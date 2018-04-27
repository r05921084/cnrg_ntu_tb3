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
MAX_DELAY = 10.


def run_MSO_model():

    dl_L = DelayLine((N_SUBCHANNELS,), SAMPLE_RATE, initial_value=0.05, max_delay=MAX_DELAY)
    dl_R = DelayLine((N_SUBCHANNELS,), SAMPLE_RATE, initial_value=0.05, max_delay=MAX_DELAY)

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

            input_node_L_probe = []
            input_node_R_probe = []
            ens_arr_add_probe = []

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


            for i in range(len(delay_values)):
                input_node_L_probe.append(nengo.Probe(input_node_L[i]))
                input_node_R_probe.append(nengo.Probe(input_node_R[i]))
                ens_arr_add_probe.append(nengo.Probe(ens_arr_add[i].output, synapse=synapse4))

        dt = 1. / SAMPLE_RATE

        # sim = nengo.Simulator(model, dt=dt)

        import nengo_dl
        sim = nengo_dl.Simulator(model, dt=dt, unroll_simulation=128)

        # import os; os.environ['PYOPENCL_CTX'] = '0'
        # import pyopencl, nengo_ocl
        # ctx = pyopencl.create_some_context()
        # sim = nengo_ocl.Simulator(model, context=ctx, dt=dt, n_prealloc_probes=1024)

        return sim

    sim = build_model()
    print sim.time, sim.n_steps
    
    rospy.Subscriber(SUB_TOPIC_NAME, AuditoryNerveImage, ani_cb)
    rospy.loginfo('"%s" starts subscribing to "%s".' % (NODE_NAME, SUB_TOPIC_NAME))

    while not rospy.is_shutdown():
        while event.wait(0.3):
            event.clear()
            steps_to_run = CHUNK_SIZE
            t2 = time.time()
            sim.run_steps(steps_to_run)
            print 'ran %d steps in %5.3f sec' % (steps_to_run, time.time() - t2)
            yet_to_run = dl_L.current_step - sim.n_steps
            print 'yet_to_run: %d steps' % yet_to_run
            if yet_to_run > MAX_DELAY * SAMPLE_RATE:
                break
        break


if __name__ == '__main__':
    try:
        rospy.init_node(NODE_NAME, anonymous=False)
        run_MSO_model()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)