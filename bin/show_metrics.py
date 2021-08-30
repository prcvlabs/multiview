#!/usr/bin/env python3

import guapi_client
import numpy as np

import argparse


if __name__ == "__main__":

	parser = argparse.ArgumentParser(formatter_class=argparse.RawDescriptionHelpFormatter,
	        description="""
Saves performance metric density plot to /output/dir/performance_metrics_<scene_name>.png

Usage: python3 plot_metrics.py <scene_name>

Examples:
	
	python3 plot_metrics.py -d /tmp california-closets_2019-10-16

	""")

	parser.add_argument("scene_name", help="Scene name from guapi")


	args=parser.parse_args()

	guapi=guapi_client.Connection()

	multiview_versions={}
	for x in guapi.list('cognitive','performance_metrics', result__scene=args.scene_name):
	    res=guapi.get('cognitive','multiview_result',x['result'])
	    try:
	        multiview_versions[res['source_version']].append(x['elapsed_time'])
	    except:
	        multiview_versions[res['source_version']]=[x['elapsed_time']]

	avg_by_version={x: {'mean':np.mean(multiview_versions[x]), 'median':np.median(multiview_versions[x])} for x in multiview_versions}

	print("multiview version", "mean elapsed time", "median elapsed time", sep='\t|\t')
	for x in avg_by_version:
	    print(x,avg_by_version[x]['mean'], avg_by_version[x]['median'], sep='\t|\t')
