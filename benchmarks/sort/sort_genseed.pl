#!/usr/bin/perl -w
#==========================================================================
# matmul_gendata.pl
#
# Author : Christopher Batten (cbatten@mit.edu)
# Date   : April 29, 2005
#
(our $usageMsg = <<'ENDMSG') =~ s/^\#//gm;
#
# Simple script which creates an input data set and the reference data
# for the matmul benchmark.
#
ENDMSG

use strict "vars";
use warnings;
no  warnings("once");
use Getopt::Long;

#--------------------------------------------------------------------------
# Command line processing
#--------------------------------------------------------------------------

our %opts;

sub usage()
{

  print "\n";
  print " Usage: matmul_gendata.pl [options] \n";
  print "\n";
  print " Options:\n";
  print "  --help  print this message\n";
  print "  --seed  random seed [1]\n";
  print "  --max   maximum output\n";
  print "$usageMsg";

  exit();
}

sub processCommandLine()
{

  $opts{"help"} = 0;
  $opts{"seed"} = 1;
  $opts{"max"}  = 64*1024;
  Getopt::Long::GetOptions( \%opts, 'help|?', 'seed:i', 'max:i') or usage();
  $opts{"help"} and usage();

}

sub main()
{

  processCommandLine();
  # srand($opts{"seed"});

  print "\#pragma once";
  print "\n\#define SEED " . int(rand(${"max"}));
 
}

main();

