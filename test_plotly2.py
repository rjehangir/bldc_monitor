import plotly
import json
import time
from math import cos,sin

with open('./config.json') as config_file:
	plotly_user_config = json.load(config_file)

username = plotly_user_config['plotly_username']
api_key = plotly_user_config['plotly_api_key']
stream_token = plotly_user_config['plotly_streaming_token']
stream_server = 'http://stream.plot.ly'

p = plotly.plotly(username, api_key)

r = p.plot([{'x':[], 'y':[], 'type': 'scatter', 'stream': {'token': stream_token, 'maxpoints': 100}}], layout={'xaxis':{'range':[-1,1]}, 'yaxis': {'range': [-1,1]}}, filename='Stream Example', fileopt='overwrite')

s = plotly.stream(stream_token)

print s

s.write({'x':0.5,'y':0.5})

i = 0
while True:
	data = {'x': cos(5*i/50.), 'y': cos(5*i/50.)*cos(i/50.)}
	s.write(data)
	i+=1
	time.sleep(0.05)

s.close()
