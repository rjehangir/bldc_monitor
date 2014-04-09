import plotly
import json
import time
import datetime
from math import cos,sin

with open('./config.json') as config_file:
	plotly_user_config = json.load(config_file)

username = plotly_user_config['plotly_username']
api_key = plotly_user_config['plotly_api_key']
stream_tokens = plotly_user_config['plotly_streaming_tokens']
stream_server = 'http://stream.plot.ly'

p = plotly.plotly(username, api_key)

#data = [{'x':[],'y':[], 'type': 'scatter', 'stream': {'token': stream_token, 'maxpoints': 100}},{'x':[1,2,3],'y':[4,5,6],'yaxis':'y2'}]

trace0 = {'x':[1,2,3,4],'y':[2,3,4,5],'type': 'scatter', 'stream': {'token': stream_tokens[0], 'maxpoints': 100}}
trace1 = {'x':[1,2,3,4],'y':[4,3,2,1],'yaxis':'y2','type': 'scatter', 'stream': {'token': stream_tokens[1], 'maxpoints': 100}}
trace2 = {'x':[1,2,3,4],'y':[4,3,2,1],'yaxis':'y3','type': 'scatter', 'stream': {'token': stream_tokens[2], 'maxpoints': 100}}
trace3 = {'x':[1,2,3,4],'y':[4,3,2,1],'yaxis':'y4','type': 'scatter', 'stream': {'token': stream_tokens[3], 'maxpoints': 100}}

xAxisStyle = {'title':'Time'}

domainHeight = 0.22
domainGap = (1.0-4*domainHeight)/3.0

layout1 = {'xaxis':xAxisStyle,
	    'yaxis':{'domain':[0,domainHeight],'title':'Voltage (V)'},
	    'yaxis2':{'domain':[domainHeight+domainGap,2*domainHeight+domainGap],'title':'Power (W)'},
	    'yaxis3':{'domain':[2*domainHeight+2*domainGap,3*domainHeight+2*domainGap],'title':'RPM'},
	    'yaxis4':{'domain':[3*domainHeight+3*domainGap,4*domainHeight+3*domainGap],'title':'Thrust (lb)'}}

p.iplot([trace0, trace1, trace2, trace3],layout=layout1,filename='Thruster-Stream',fileopt='overwrite')

s0 = plotly.stream(stream_tokens[0])
s1 = plotly.stream(stream_tokens[1])
s2 = plotly.stream(stream_tokens[2])
s3 = plotly.stream(stream_tokens[3])

s0.write({'x':0.5,'y':0.5})
s1.write({'x':0.5,'y':0.5})
s2.write({'x':0.5,'y':0.5})
s3.write({'x':0.5,'y':0.5})

i = 0
while True:
	timeStamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
	timeStamp = i
	trace0 = {'x': timeStamp, 'y': cos(5*i/50.)*cos(i/50.)}
	trace1 = {'x': timeStamp, 'y': cos(5*i/50.)*cos(i/50.)}
	trace2 = {'x': timeStamp, 'y': -cos(5*i/50.)*cos(i/50.)}
	trace3 = {'x': timeStamp, 'y': -cos(5*i/50.)*cos(i/50.)}
	s0.write(trace0)
	s1.write(trace1)
	s2.write(trace2)
	s3.write(trace3)
	i+=1
	time.sleep(0.1)

s0.close()
s1.close()
