import nengo
import nengo_dl
import numpy as np

n_steps = 5
data_dim = 1
mini_batch = 3

def out_cb(t, x):
    print t, x


feeds = np.arange(mini_batch*n_steps*data_dim).reshape((mini_batch, n_steps, data_dim)) + np.arange(mini_batch)[:, None, None]


with nengo.Network() as net:
    node = nengo.Node([0], size_in=0, size_out=data_dim)
    p = nengo.Probe(node)
    out_node = nengo.Node(output=out_cb, size_in=data_dim, size_out=0)
    nengo.Connection(node, out_node, synapse=None)

with nengo_dl.Simulator(net, minibatch_size=mini_batch) as sim:
    sim.run_steps(n_steps, input_feeds={node: feeds},)

    print sim.data[p]