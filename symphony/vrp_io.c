/*===========================================================================*/
/*                                                                           */
/* This file is part of a demonstration application for use with the         */
/* SYMPHONY Branch, Cut, and Price Library. This application is a solver for */
/* the Vehicle Routing Problem and the Traveling Salesman Problem.           */
/*                                                                           */
/* (c) Copyright 2000-2007 Ted Ralphs. All Rights Reserved.                  */
/*                                                                           */
/* This application was developed by Ted Ralphs (ted@lehigh.edu)             */
/*                                                                           */
/* This software is licensed under the Eclipse Public License. Please see    */
/* accompanying file for terms.                                              */
/*                                                                           */
/*===========================================================================*/

/* system include files */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

/* SYMPHONY include files */
#include "sym_macros.h"
#include "sym_types.h"
//#include "sym_master_u.h"   // may not need this file for me.
//#include "sym_lp_params.h"  // may not need this file for me.

/* VRP include files */
#include "vrp_io.h"
#include "vrp_types.h"
#include "vrp_const.h"
#include "vrp_macros.h"
#include "compute_cost.h"
//#include "small_graph.h"    // may not need this file for me.

/* I add these macros in 11/4/2013 */
#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#define ISIZE sizeof(int)

/*===========================================================================*/

/*===========================================================================*\
 * This file contains the user I/O functions for the master process.
 \*===========================================================================*/

/*===========================================================================*\
 * This first function reads in the data instance.
 \*===========================================================================*/

void vrp_io(vrp_problem *vrp, char *infile)
{
	static char keywords[KEY_NUM][22] = {
		"NAME", 
		"NAME:",                 /* This section lists the names of the */
		"TYPE",                  /* possible fields in the data file    */
		"TYPE:",
		"COMMENT",
		"COMMENT:",
		"DIMENSION",
		"DIMENSION:",
		"CAPACITY",
		"CAPACITY:",
		"EDGE_WEIGHT_TYPE",
		"EDGE_WEIGHT_TYPE:",
		"EDGE_WEIGHT_FORMAT", 
		"EDGE_WEIGHT_FORMAT:", 
		"DISPLAY_DATA_TYPE",
		"DISPLAY_DATA_TYPE:",
		"EDGE_WEIGHT_SECTION", 
		"EDGE_WEIGHT_SECTION:", 
		"DISPLAY_DATA_SECTION", 
		"DISPLAY_DATA_SECTION:",
		"NODE_COORD_SECTION",
		"NODE_COORD_SECTION:",
		"NODE_COORD_TYPE",
		"NODE_COORD_TYPE:",
		"DEPOT_SECTION",
		"DEPOT_SECTION:",
		"CAPACITY_VOL",
		"CAPACITY_VOL:",
		"DEMAND_SECTION",
		"DEMAND_SECTION:",
		"TIME_WINDOW_SECTION",
		"TIME_WINDOW_SECTION:",
		"STANDTIME_SECTION",
		"STANDTIME_SECTION:",
		"PICKUP_SECTION",
		"PICKUP_SECTION:",
		"EOF",
		"EOF.",
		"NUMBER_OF_TRUCKS",
		"NUMBER_OF_TRUCKS:",
		"",
		"",
		"NO_MORE_TYPE"
	};
	
#define NCTYPE_NUM 3
	
	static char nctypes[NCTYPE_NUM][14] = {
		"TWOD_COORDS",
		"THREED_COORDS",     /*This section lists the possible node*/
		"NO_COORDS"          /*coordinate data types               */
	};
	
#define WTYPE_NUM 10
	
	static char wtypes[WTYPE_NUM][9] = {
		"EXPLICIT",
		"EUC_2D",            /*This is a list of the possible data types for */
		"EUC_3D",            /*edge weights                                  */
		"MAX_2D",
		"MAX_3D",
		"MAN_2D",
		"MAN_3D",
		"CEIL_2D",
		"GEO",
		"ATT"
	};
	
#define WFORMAT_NUM 9
	
	static char wformats[WFORMAT_NUM][20] = {
		"UPPER_ROW",
		"LOWER_ROW",          /*This is a list of the possible formats that*/
		"UPPER_DIAG_ROW",     /*the edge weight matrix could be given in   */
		"LOWER_DIAG_ROW",     /*if it is given explicitly                  */
		"UPPER_COL",
		"LOWER_COL",
		"UPPER_DIAG_COL",
		"LOWER_DIAG_COL",
		"FULL_MATRIX"
	};
	
#define DTYPE_NUM 3
	
	static char dtypes[DTYPE_NUM][14] = {
		"COORD_DISPLAY",
		"TWOD_DISPLAY",     /*This is a list of the various display data*/
		"NO_DISPLAY"        /*types                                     */
	};
	
	char line[LENGTH], line1[LENGTH], key[30], tmp[80];
	int wformat=-1, dtype=-1, nctype=-1;
	double fdummy;
	int i, j = 0;
	int l, m, *coef2;
	FILE *f;
	int node;
	double deg, min, coord_x, coord_y, coord_z;
	double x, y;
	int capacity_vol = FALSE;
	int k;
	register int vertnum = 0;
	distances *dist = &vrp->dist;
	
	if (!strcmp(infile, "")){
		printf("\nVrp I/O: No problem data file specified\n\n");
		exit(1);
	}
	
	if ((f = fopen(infile, "r")) == NULL){
		fprintf(stderr, "Vrp I/O: file '%s' can't be opened\n", infile);
		exit(1);
	}
	
	/*This loop reads in the next line of the data file and compares it
	 to the list of possible keywords to determine what data will follow.
	 It then reads the data into the appropriate field and iterates */
	
	while(NULL != fgets( line1, LENGTH, f)){
		strcpy(key,"");
		sscanf(line1,"%s",key); /*read in next keyword*/
		
		for (k = 0; k < KEY_NUM; k++) /*which data field comes next?*/
			if (strcmp(keywords[k], key) == 0) break;
		
		if (k == KEY_NUM){
			continue;
			fprintf(stderr, "Unknown keyword! bye.\n");
			exit(1); /*error check for acceptable data field*/
		}
		
		k >>= 1; /* This is a bit shift operation that divides k by 2    */
		/* since in the list of keywords, there are two possible*/
		/* formats for the keyword                              */
		
		if (strchr(line1,':')){
			strcpy(line, strchr(line1, ':')+1);
		}
		switch (k){
				
			case 0: /* NAME */
				if (!sscanf(line, "%s", vrp->name))
					fprintf(stderr, "\nVrp I/O: error reading NAME\n\n");
				//printf("PROBLEM NAME: \t\t%s\n", vrp->name);
				break;
			case 1 : /*TYPE*/
				sscanf(line, "%s", tmp);
				if (strcmp("CVRP", tmp) != 0){
					if (strcmp("TSP", tmp) == 0){
						vrp->par.tsp_prob = TRUE;
						/*__BEGIN_EXPERIMENTAL_SECTION__*/
					}else if (strcmp("BPP", tmp) == 0){
						vrp->par.bpp_prob = TRUE;
						/*___END_EXPERIMENTAL_SECTION___*/
					}else{
						fprintf(stderr, "This is not a recognized file type!\n");
						exit(1);
					}
				}
				//printf("TYPE: \t\t\t%s\n", tmp);
			case 2 : /*COMMENT*/
#if 0
				if (!strncpy(tmp, line, 80))
					fprintf(stderr, "\nVrp I/O: error reading COMMENT\n\n");
				printf("DESCRIPTION: \t\t%s\n", tmp);
#endif
				break;
			case 3 : /* DIMENSION */
				if (!sscanf(line, "%i", &k)){
					fprintf(stderr, "Vrp I/O: error reading DIMENSION\n\n");
					exit(1);
				}
				vertnum = vrp->vertnum = (int) k;
				vrp->edgenum = (int) vertnum * (vertnum - 1)/2;
				//printf("DIMENSION: \t\t%i\n", k);
				break;
			case 4 : /*CAPACITY*/
				if (!sscanf(line, "%i", &k)){
					fprintf(stderr, "Vrp I/O: error reading CAPACITY\n\n");
					exit(1);
				}
				vrp->capacity = (int) k;
				break;
			case 5 : /* EDGE_WEIGHT_TYPE */
				sscanf(line, "%s", tmp);
				for (dist->wtype = 0; dist->wtype < WTYPE_NUM; (dist->wtype)++)
					if (strcmp(wtypes[dist->wtype], tmp) == 0) break;
				if (dist->wtype == WTYPE_NUM) {
					fprintf(stderr, "Unknown weight type : %s !!!\n", tmp);
					exit(1);
				}
				break;
			case 6 : /* EDGE_WEIGHT_FORMAT */
				sscanf(line, "%s", tmp);
				for (wformat = 0; wformat < WFORMAT_NUM; wformat++)
					if (strcmp(wformats[wformat], tmp) == 0) break;
				if (wformat == WFORMAT_NUM) {
					fprintf(stderr, "Unknown weight type : %s !!!\n", tmp);
					exit(1);
				}
				break;
			case 7 : /* DISPLAY_DATA_TYPE */
				sscanf(line, "%s", tmp);
				for (dtype = 0; dtype < DTYPE_NUM; dtype++)
					if (strcmp(dtypes[dtype], tmp) == 0) break;
				if (dtype == DTYPE_NUM) {
					fprintf(stderr, "Unknown display type : %s !!!\n", tmp);
					exit(1);
				}
				break;
			case 8: /* EDGE_WEIGHT_SECTION */
				/*------------------------break if not EXPLICIT -*/
				if (dist->wtype != _EXPLICIT) break; 
				dist->cost = (int *) malloc (vrp->edgenum*sizeof(int));
				switch (wformat){
					case 1 : /* LOWER_ROW */
					case 4 : /* UPPER_COL */
					case 3 : /* LOWER_DIAG_ROW */
					case 6 : /* UPPER_DIAG_COL */
						for (i=0, coef2=dist->cost; i<vertnum; i++){
							for (j=0; j<i; j++, coef2++){
								if (!fscanf(f,"%lf", &fdummy)){
									fprintf(stderr, "Not enough data -- DIMENSION or "
											"EDGE_WEIGHT_TYPE declared wrong\n");
									exit(1);
								}
								else {
									*coef2 = (int) fdummy;
								}
							}
							if ((wformat==3 || wformat==6) && 
								!fscanf(f,"%lf", &fdummy)){
								fprintf(stderr, "Not enough data -- DIMENSION or "
										"EDGE_WEIGHT_TYPE declared wrong\n");
								exit(1);
							}
						}
						if (fscanf(f,"%lf", &fdummy)){
							fprintf(stderr, "Too much data -- DIMENSION or "
									"EDGE_WEIGHT_TYPE declared wrong\n");
							exit(1);
						}
						break;
					case 0 : /* UPPER_ROW */
					case 5 : /* LOWER_COL */
					case 2 : /* UPPER_DIAG_ROW */
					case 7 : /* LOWER_DIAG_COL */
						for (i=0, coef2=dist->cost; i<vertnum; i++){
							if (wformat==2 || wformat==7) 
								if (!fscanf(f,"%lf", &fdummy)){
									fprintf(stderr, "Not enough data -- DIMENSION or "
											"EDGE_WEIGHT_TYPE declared wrong");
									exit(1);
								}
							for (j=i+1; j<vertnum; j++){
								if (!fscanf(f,"%lf", &fdummy)){
									fprintf(stderr, "Not enough data -- DIMENSION or "
											"EDGE_WEIGHT_TYPE declared wrong");
									exit(1);
								}
								else coef2[j*(j-1)/2+i] = (int) fdummy;
							}
						}
						if (fscanf(f,"%lf", &fdummy)){
							fprintf(stderr, "Too much data -- DIMENSION or "
									"EDGE_WEIGHT_TYPE declared wrong\n");
							exit(1);
						}
						break;
					case 8 : /* FULL_MATRIX */
						for (i=0, coef2=dist->cost; i<vertnum; i++){
							for (j=0; j<=i; j++)
								if(!fscanf(f,"%lf", &fdummy)){
									fprintf(stderr, "Not enough data -- DIMENSION or "
											"EDGE_WEIGHT_TYPE declared wrong");
									exit(1);
								}
							for (j=i+1; j<vertnum; j++){
								if(!fscanf(f,"%lf", &fdummy)){
									fprintf(stderr, "Not enough data -- DIMENSION or "
											"EDGE_WEIGHT_TYPE declared wrong");
									exit(1);
								}
								coef2[j*(j-1)/2+i] = (int) fdummy;
							}
						}
						if (fscanf(f,"%lf", &fdummy)){
							fprintf(stderr, "Too much data -- DIMENSION or "
									"EDGE_WEIGHT_TYPE declared wrong\n");
							exit(1);
						}
						break;
                    default: break;
				}
				break;
			case 9 : /* DISPLAY_DATA_SECTION */
				/*--------------------- break if NO_DISPLAY -*/
				if (dtype != 1){
					fprintf(stderr, "DISPLAY_DATA_SECTION exists"
							"but not TWOD_DISPLAY!\n");
					exit(1);
				}
				/* posx, posy -*/
				vrp->posx = (int *) malloc (vertnum*sizeof(int));
				vrp->posy = (int *) malloc (vertnum*sizeof(int));
				for (i=0; i<vertnum; i++){
					if ((k = fscanf(f,"%i%lf%lf", &node, &x, &y)) != 3){
						fprintf(stderr, "\nVrp I/O: error reading DISPLAY_DATA\n");
						break;
					}
					vrp->posx[node-1] = (int)(x + 0.5);
					vrp->posy[node-1] = (int)(y + 0.5);
				}
				if (fscanf(f,"%lf", &fdummy)){
					fprintf(stderr, "\nVrp I/O: too much display data\n");
					break;
				}
				break;
			case 10 : /* NODE_COORD_SECTION */
				if (nctype == -1) nctype = 0;  /*if not given: TWOD_COORDS*/
				if (dtype == -1 && ((dist->wtype == _EUC_2D) || /*display type*/
									(dist->wtype == _MAX_2D) ||  /*not defd yet*/
									(dist->wtype == _MAN_2D)   ))/*&& can disp.*/
					dtype = 0;                               /* COORD_DISPLAY */
				if (dtype == 0){
					vrp->posx = (int *) malloc (vertnum*sizeof(int));
					vrp->posy = (int *) malloc (vertnum*sizeof(int));
				}
				dist->coordx = (double *) malloc (vertnum*sizeof(double));
				dist->coordy = (double *) malloc (vertnum*sizeof(double));
				if (nctype == 1)
					dist->coordz = (double *) malloc (vertnum*sizeof(double));
				for (i=0; i<vertnum; i++){
					if (nctype == 0)          /* TWOD_COORDS */
						if (fscanf(f,"%i%lf%lf", &node, &coord_x, &coord_y) != 3){
							fprintf(stderr, "\nVrp I/O: error reading NODE_COORD\n\n");
							exit(1);
						}
					if (nctype == 1)          /* THREED_COORDS */
						if (fscanf(f,"%i%lf%lf%lf", &node, &coord_x, &coord_y,
								   &coord_z) != 4){
							fprintf(stderr, "\nVrp I/O: error reading NODE_COORD\n\n");
							exit(1);
						}
					dist->coordx[node-1] = coord_x;
					dist->coordy[node-1] = coord_y;
					/*since position is an integer and coord is a double, I must
					 round off here if dtype is EXPLICIT*/
					if (dtype == 0){
						vrp->posx[node-1] = (int)coord_x;
						vrp->posy[node-1] = (int)coord_y;
					}
					if (nctype == 1) dist->coordz[node-1] = coord_z;
					if (dist->wtype == _GEO){ /* GEO */
						/*--- latitude & longitude for node ------------*/
						deg = (int)(dist->coordx[node-1]);
						min = dist->coordx[node-1] - deg;
						dist->coordx[node-1] = MY_PI * (deg + 5.0*min/3.0 ) / 180.0;
						deg = floor(dist->coordy[node-1]);
						min = dist->coordy[node-1] - deg;
						dist->coordy[node-1] = MY_PI * (deg + 5.0*min/3.0 ) / 180.0;
					}
				}
				if (fscanf(f,"%i%lf%lf%lf", &node, &coord_x, &coord_y, &coord_z)){
					fprintf(stderr, "\nVrp I/O: too much data in NODE_COORD\n\n");
					exit(1);
				}
				break;
			case 11: /* NODE_COORD_TYPE */
				sscanf(line, "%s", tmp);
				for (nctype = 0; nctype < NCTYPE_NUM; nctype++)
					if (strcmp(nctypes[nctype], tmp) == 0) break;
				if (nctype == NCTYPE_NUM) {
					fprintf(stderr, "Unknown node_coord_type : %s !!!\n", tmp);
					exit(1);
				}
				break;
			case 12: /*DEPOT_SECTION*/
				fscanf(f, "%i", &k);
				if (k != 1){
					fprintf(stderr, "Error in data: depot must be node 1");
					exit(1);
				}
				vrp->depot = k - 1;
				while (-1 != k) fscanf(f, "%i", &k);
				break;
			case 13: /*CAPACITY_VOL*/
				sscanf(line, "%i", &k);
				capacity_vol = TRUE;
				break;
			case 14: /*DEMAND_SECTION*/
				vrp->demand = (int *) malloc(vertnum*sizeof(int));
				for (i = 0; i < vertnum; i++){
					if (capacity_vol){
						if (fscanf(f, "%i%i%i", &k, &l, &m) != 3){
							fprintf(stderr,"\nVrp I/O: error reading DEMAND_SECTION\n\n");
							exit(1);
						}
					}
					else if (fscanf(f, "%i%i", &k, &l) != 2){
						fprintf(stderr, "\nVrp I/O: error reading DEMAND_SECTION\n\n");
						exit(1);
					}
					vrp->demand[k-1] = l;
					vrp->demand[0] += l;
				}
				if (fscanf(f, "%i%i", &k, &l)){
					fprintf(stderr, "\nVrp I/O: too much data in DEMAND_SECTION\n\n");
					exit(1);
				}
				break;
			case 15: /*TIME_WINDOW_SECTION*/  /*These sections are not used*/
				while (fscanf(f, "%d %*d:%*d %*d:%*d", &k));
				break;
			case 16: /*STANDTIME_SECTION*/
				while (fscanf(f, "%d%*d", &k));
				break;
			case 17: /*PICKUP_SECTION*/       
				while (fscanf(f, "%d%*d%*d", &k));
				break;
			case 18: /*  EOF  */
				break;
			case 19: /*  NUMBER_OF_TRUCKS  */
				if (!sscanf(line, "%i", &k)){
					fprintf(stderr, "Vrp I/O: error reading NO_OF_TRUCKS\n\n");
					exit(1);
				}
				vrp->numroutes = (int) k;
			default:
				break;
		}
	}
	
	if (f != stdin)
		fclose(f);
	
	/*calculate all the distances explcitly and then use distance type EXPLICIT*/
	
	/*__BEGIN_EXPERIMENTAL_SECTION__*/
	if (vrp->par.bpp_prob){
		dist->cost = (int *) calloc (vrp->edgenum, sizeof(int));
		for (i = 1, k = 0; i < vertnum; i++){
			for (j = 0; j < i; j++){
				dist->cost[k++] = vrp->demand[i]+vrp->demand[j];
			}
		}
		dist->wtype = _EXPLICIT;
	}
	/*___END_EXPERIMENTAL_SECTION___*/
	if (dist->wtype != _EXPLICIT){
		dist->cost = (int *) calloc (vrp->edgenum, sizeof(int));
		for (i = 1, k = 0; i < vertnum; i++){
			for (j = 0; j < i; j++){
				dist->cost[k++] = ICOST(dist, i, j);
			}
		}
		dist->wtype = _EXPLICIT;
	}
	
	if (vrp->par.tsp_prob){
		vrp->capacity = vertnum;
		vrp->numroutes = 1;
		vrp->demand = (int *) malloc (vertnum * ISIZE);
		vrp->demand[0] = vertnum;
		for (i = vertnum - 1; i > 0; i--)
			vrp->demand[i] = 1;
		vrp->cg_par.tsp_prob = TRUE;
		if (!vrp->cg_par.which_tsp_cuts)
			vrp->cg_par.which_tsp_cuts = ALL_TSP_CUTS;
	}
}

/*===========================================================================*/

