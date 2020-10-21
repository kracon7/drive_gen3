import sys
import os
import argparse
import pyaudio
import wave
import time as tm
import numpy as np
# import matplotlib.pyplot as plt
# import librosa
# import librosa.display

def record(time, out_path):
	CHUNK = 1024
	FORMAT = pyaudio.paInt16
	CHANNELS = args.n_channel
	RATE = 44100
	RECORD_SECONDS = time
	WAVE_OUTPUT_FILENAME = out_path

	p = pyaudio.PyAudio()

	stream = p.open(format=FORMAT,
	                channels=CHANNELS,
	                rate=RATE,
	                input=True,
	                frames_per_buffer=CHUNK)

	print("{}: * recording".format(tm.time()))

	frames = []

	for i in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
	    data = stream.read(CHUNK)
	    frames.append(data)

	print("* done recording")

	stream.stop_stream()
	stream.close()
	p.terminate()

	wf = wave.open(WAVE_OUTPUT_FILENAME, 'wb')
	wf.setnchannels(CHANNELS)
	wf.setsampwidth(p.get_sample_size(FORMAT))
	wf.setframerate(RATE)
	wf.writeframes(b''.join(frames))
	wf.close()

def analyze_audio(audio_path, feature_wf):

	n_fft = 2048
	hop = 1/4
	threshold = 0.003

	x_1 , sr_1 = librosa.load(audio_path)
	# frequency analysis
	X_1 = librosa.stft(x_1)
	
	score = abs(X_1).T @ feature_wf
	score[score<5] = 0
	return np.sum(score)


def main(args):
	out_path = os.path.join(args.out_dir, '{}_{}.wav'.format(args.epoch, args.episode))
	record(args.time, out_path)
	feature_wf = np.load('./spectrum.npy')
	# reward = analyze_audio(out_path, feature_wf)
	# f = open('/home/jc/logs/real_robot/reward.txt', 'a+')
	# f.write('{}  {}  {}\n'.format(args.epoch, args.episode, reward))
	# f.close()


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Record audio signal')
    parser.add_argument('--out_dir', default='/home/jc/logs/realrobot', help='output file path')
    parser.add_argument('-t', '--time', default=5, type=int, help='lenght of time recording, in seconds')
    parser.add_argument('--n_channel', default=1, type=int, help='number of channels') 
    parser.add_argument('--epoch', type=int, help='number of epoch')
    parser.add_argument('--episode', type=int, help='number of episode in and epoch')   

    args = parser.parse_args()
    
    main(args)
