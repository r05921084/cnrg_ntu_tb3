import subprocess
import wave
import os
import numpy as np

class IPEM_subproc:
    def __init__(self, inInputFileName='input.wav', inInputFilePath='.',
                 inOutputFileName='output.ani', inOutputFilePath='.',
                 inNumOfChannels=40, inFirstFreq=2.0, inFreqDist=0.5,
                 skip_check=False, sample_frequency=None):

        if not skip_check:
            if inInputFileName.split('.')[-1] != 'wav':
                raise ValueError('Only support *.wav file.')

            loaded_wav = wave.open(inInputFileName)

            if loaded_wav.getnchannels() != 1:
                raise ValueError('Only support mono channel audio.')

            if sample_frequency is None:
                sample_frequency = loaded_wav.getframerate()

        else:
            if sample_frequency is None:
                raise ValueError('Must specify "sample_frequency" when "skip_check=True".')

        self.inInputFileName = inInputFileName
        self.inInputFilePath = inInputFilePath
        self.inOutputFileName = inOutputFileName
        self.inOutputFilePath = inOutputFilePath
        self.inNumOfChannels = inNumOfChannels
        self.inFirstFreq = inFirstFreq
        self.inFreqDist = inFreqDist
        self.inSoundFileFormat = 'wav'
        self.sample_frequency = sample_frequency
        self.ani_path = self.inOutputFilePath + '/' + self.inOutputFileName

    def process(self):
        ani, log_str = None, None
        try:
            log_str = subprocess.check_output (['./IPEMAuditoryModelConsole',
                                                '-nc', str(self.inNumOfChannels), 
                                                '-f1', str(self.inFirstFreq),
                                                '-fd', str(self.inFreqDist),
                                                '-if', self.inInputFileName, 
                                                '-id', self.inInputFilePath,
                                                '-of', self.inOutputFileName, 
                                                '-od', self.inOutputFilePath,
                                                '-fs', str(self.sample_frequency), 
                                                '-ff', self.inSoundFileFormat])
            # print 'call to ./IPEMAuditoryModelConsole returned in %f sec' % (time.time() - t1)
            # print log_str
        except subprocess.CalledProcessError as e:
            print 'CalledProcessError'
            print 'Command:', ' '.join(e.cmd)
            print 'Returncode:', e.returncode
            print e.output
        else:
            ani = np.loadtxt(self.ani_path)
        # finally:
            # try:
                # os.remove('./outfile.dat')
                # os.remove('./omef.dat')
                # os.remove('./lpf.dat')
                # os.remove('./filters.dat')
                # os.remove('./FilterFrequencies.txt')
                # os.remove('./eef.dat')
                # os.remove('./decim.dat')
            #     os.remove(self.ani_path)
            # except OSError:
            #     pass

        return ani, log_str


if __name__ == '__main__':
    ipem_L = IPEM_subproc(inInputFileName='input_L.wav', inOutputFileName='output_L.ani')
    ipem_R = IPEM_subproc(inInputFileName='input_R.wav', inOutputFileName='output_R.ani')

    ani_L, log_L = ipem_L.process()
    ani_R, log_R = ipem_R.process()

    print ani_L.shape, ani_R.shape
    print log_L
    print log_R