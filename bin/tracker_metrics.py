import guapi_client
import argparse
import boto3
import os
import json
import numpy as np
from matplotlib import pyplot as plt
import seaborn as sns

def getPathLength(path):
	_path_length=0
	for i in range(1,len(path)):
		if path[i][0]!=path[i-1][0] or path[i][1]!=path[i-1][1]:
			_path_length+=np.linalg.norm((path[i][0]-path[i-1][0], path[i][1]-path[i-1][1]))
	return _path_length

def getPathDisplacement(path):
	return np.linalg.norm((path[0][0]-path[-1][0], path[0][1]-path[-1][1]))

def getPathAvgVelocity(path):
	return getPathDisplacement(path)/len(path)

def getPathAvgSpeed(path):
	return getPathLength(path)/len(path)

def getSmoothPath(path, window=5):
	path=np.array(path)
	if len(path)<=window:
		return path
	else:
		smooth_path=[]
		smooth_path.append((path[0,0],path[0,1]))
		for i in range(len(path)-window):
			smooth_path.append((np.mean(path[i:i+window,0]),np.mean(path[i:i+window,1])))
		smooth_path.append((path[-1,0],path[-1,1]))
		return smooth_path

def informationContent(path):
	path_point_counts={}
	for x in path:
		try:
			path_point_counts[(x[0],x[1])]+=1
		except KeyError:
			path_point_counts[(x[0],x[1])]=1
	path_dist={x: path_point_counts[x]/len(path) for x in path_point_counts}

	return -1*sum([x*np.log(x) for x in path_dist.values()])


def gazeAgainstDirection(path, smoothing_window=45):
	# This function estimates the reasonability of a gaze direction given motion of a path.
	# If there is a high gaze against direction rate, then the two values don't seem to be in accordance.
	thetas=[]
	for i in range(0,len(path)-smoothing_window-1):
		if not path[i][3]:
			continue
		gaze_dir=path[i][3]
		dx=np.mean(np.array(path)[i+1:i+smoothing_window+1,0])-path[i][0]
		dy=np.mean(np.array(path)[i+1:i+smoothing_window+1,1])-path[i][1]
		if dx==0 and dy==0:
			pass
		else:
			path_length=np.linalg.norm((dx,dy))
			dy_gaze=np.sin(gaze_dir)
			dx_gaze=np.cos(gaze_dir)
			theta=np.arccos((dx_gaze*dx+dy_gaze*dy)/path_length)
			thetas.append(theta)
	return thetas


def countMissingValues(track_results, distance_tolerance=50, frame_tolerance=45):
	#This function can take in a list of paths, but if they are not in the same epoch,
	#then the results are not interpretable. Distance tolerance is cm.
	#Frame tolerance is number of frames.

	start_points={}
	end_points={}
	standing_points=[]
	total_points=0
	for path in track_results:
		if len(path)>1:
			try:
				start_points[path[0][2]].append((path[0][0],path[0][1]))
				end_points[path[-1][2]].append((path[-1][0],path[-1][1]))
			except KeyError:
				start_points[path[0][2]]=[(path[0][0],path[0][1])]
				end_points[path[-1][2]]=[(path[-1][0],path[-1][1])]
		total_points+=len(path)

	missing_values=0

	for i in range(0,max(end_points)+1):
		try:
			for end_point in end_points[i]:
				for j in range(i+1,i+frame_tolerance+1):
					if j in start_points:
						for start_point in start_points[j]:
							if np.linalg.norm((start_point[0]-end_point[0],start_point[1]-end_point[1]))<distance_tolerance:
								missing_values+=(j-i)
		except KeyError:
			pass
	return missing_values



def drawPathMap(paths):
	fig=plt.figure()
	fig.set_size_inches(8,8)
	plt.gca().set_aspect('equal')
	for path in paths:
		x=[p[0] for p in path]
		y=[p[1] for p in path]
		plt.plot(x,y, ms=2, alpha=0.05)
	fig.savefig('path_map.png',dpi=300)
	plt.close(fig)

def drawStartEndPoints(paths):
	fig=plt.figure()
	fig.set_size_inches(8,8)
	plt.gca().set_aspect('equal')
	x_start,y_start=[],[]
	x_end,y_end=[],[]
	for path in paths:
		x_start.append(path[0][0])
		y_start.append(path[0][1])
		x_end.append(path[-1][0])
		y_end.append(path[-1][1])
	plt.scatter(x_start,y_start,c='green',alpha=0.005)
	plt.scatter(x_end,y_end,c='red',alpha=0.005)
	fig.savefig('start_end.png',dpi=300)
	plt.close(fig)


if __name__=="__main__":
	parser=argparse.ArgumentParser(description="""Calculate and print benchmarking metrics for a specific version of multiview.""",epilog=r'''Example:
  python3 ./tracker_metrics.py \
    --scene museum_2019-07-01_v1 \
    --source_version  3.7.2:commit:9f92f878977a5679c65c284fa5cad2e30bdeaaa2 \
    --limit 15 \
    --path_map \
    --start_end \
    --jointplot \
    --gaze_thetas''', formatter_class=argparse.RawTextHelpFormatter)
	parser.add_argument("--scene", type=str)
	parser.add_argument("--limit", type=int, default=10,
		help="Maximum number of tracks to download. Defaults to 10.")
	parser.add_argument("--source_version",type=str)
	parser.add_argument("--path_map", action="store_true",
		help="Save a visualization that traces out the entire tracks to `path_map.png`.\nLong, straight lines are indicative of spatial jump in tracks.")
	parser.add_argument("--start_end", action="store_true",
		help="Save a vsualization of the starting and ending points to `start_end.png`.\nVery dark regions are indicative of broken tracks.")
	parser.add_argument("--jointplot",action="store_true",
		help="Save a jointplot of path lengths vs path displacements to `jointplot.png`")
	parser.add_argument("--gaze_thetas",action="store_true",
		help="Save a distribution plot of angular distances between gaze\nand movement to `gaze_vs_velocity_direction.png`")

	args = parser.parse_args()

	guapi=guapi_client.Connection()
	s3=boto3.client('s3')
	try:
		os.mkdir('tmp_data')
	except FileExistsError:
		pass
	a=0
	for x in guapi.list('cognitive','multiview_result',
		type='tracks',
		source_version=args.source_version,
		scene=args.scene):
		if a==args.limit:
			break
		filepath=x['key'].split('/')[-1]
		print('Downloading '+filepath+'...')
		with open('tmp_data/'+filepath, 'wb') as f:
			s3.download_fileobj(x['bucket'],x['key'],f)
		a+=1

	paths=[]

	missing_points=0

	for file in os.listdir('./tmp_data'):
		if file.endswith(".json"):
			with open('./tmp_data/'+file, 'r') as f:
				data=json.loads(f.read())
		single_track_paths=[data['track_results']['tracks'][i]['path'] for i in range(len(data['track_results']['tracks']))]
		missing_points+=countMissingValues(single_track_paths)
		paths.extend(single_track_paths)

	path_displacements=[]
	path_lengths=[]
	path_velocities=[]
	gaze_thetas=[]

	point_in_non_displaced_paths=0
	less_than_one_meter_displaced=0
	less_than_50cm_displaced=0

	total_points=0

	for path in paths:
		total_points+=len(path)
		path_displacement=getPathDisplacement(path)
		path_length=getPathLength(path)
		path_velocity=getPathAvgVelocity(path)
		gaze_theta=gazeAgainstDirection(path)

		if path_displacement==0:
			point_in_non_displaced_paths+=len(path)

		if path_displacement<10:
			less_than_one_meter_displaced+=len(path)

		if path_displacement<5:
			less_than_50cm_displaced+=len(path)

		path_displacements.append(path_displacement)
		path_lengths.append(path_length)
		path_velocities.append(path_velocity)
		gaze_thetas.extend(gaze_theta)

	print("""\n\n\nThese proportional metrics represent measures of false positives
or tracks that were stitched incorrectly. Higher values indicate a higher
proportion of false positives or sporadic stitching.""")
	print("\nProportion of tracks with zero total displacement:\n",
			sum([1 for x in path_displacements if x==0])/len(path_displacements),'\n')

	print("Proportion of tracks with total displacement less than 50cm:\n",
			sum([1 for x in path_displacements if x<5])/len(path_displacements),'\n')
	
	print("Proportion of tracks with total displacement less than one meter:\n",
			sum([1 for x in path_displacements if x<10])/len(path_displacements),'\n')	

	print("""Proportion of all points that are within tracks with
zero displacement:\n""",
			point_in_non_displaced_paths/total_points,"\n")

	print("""Proportion of all points that are within tracks with
less than 50cm total displacement:\n""",
			less_than_50cm_displaced/total_points,"\n")

	print("""Proportion of all points that are within tracks with
less than one meter total displacement:\n""",
			less_than_one_meter_displaced/total_points,"\n\n\n")


	print("""This next metric demonstrates a discordance between gaze and track
movement direction. A higher value indicates less dependable performance.""")
	print("""Proportion of points with gaze in which the track's
movement direction does not correspond to the gaze direction:\n""",
			sum([1 for x in gaze_thetas if x>(2*np.math.pi/3)])/len(gaze_thetas),"\n")

	print("""Recall can be estimated by tallying the number of frames missing between
tracks that end then quickly resume. This is a rough estimate and does not tally
points in which there was no attributed track anywhere nearby.""")
	print("""Roughly estimated recall:\n""", total_points/(total_points+missing_points),"\n")

	if args.jointplot:
		sns_plot=sns.jointplot(np.log1p(path_lengths),np.log1p(path_displacements),alpha=0.05)
		sns_plot.set_axis_labels('ln(1+path_length)','ln(1+path_displacement)')
		fig=sns_plot.fig
		fig.set_size_inches(8,8)
		fig.savefig('jointplot.png')
		plt.close(fig)
		print("Saved joint distribution plot of ln(1+path_length) and ln(1+path_displacement) to `jointplot.png`.\n")

	if args.gaze_thetas:
		sns_plot=sns.distplot(gaze_thetas)
		fig=sns_plot.get_figure()
		fig.set_size_inches(8,8)
		fig.savefig('gaze.png')
		plt.close(fig)
		print("Saved gaze distribution chart to `gaze.png`.\n")

	if args.path_map:
		drawPathMap(paths)
		print("Saved path map to `path_map.png`.\n")
	
	if args.start_end:
		drawStartEndPoints(paths)
		print("Saved starting and ending point scatter plot to `start_end.png`.\n")
