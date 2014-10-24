__author__ = 'santiago'

from openravepy import *

env=Environment()
env.SetViewer('qtcoin')

with env:
    print 'Santiago'
