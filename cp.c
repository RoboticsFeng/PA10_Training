/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *\
 * cp.c  : Command Processing Server                                             *
 *                                                                               *
 * CAUTION  : THIS PROGRAM IS ONLY FOR  " Q N X "                                *	
 *                                                                               *
 * Auther   : Takahiro Baba                                                      *
 * Date     : 2002/10/28                                                         *
 * Version  : 0.5                                                                *
 * comment  : This server can receive, process and decode user commands          *
 *            sended by cpc(command processing client program).                  *
 * See Also : Main Program (usually calling in main())                           *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "include.h"

extern void init(void);
extern void start(void);
extern void fin(void);

extern void allzero(void);
extern void sample(void);
extern void set_angles(double angles[8]);
extern void endpoint_moveto(double input[8]);
extern void lineinit(void);
extern void line(void);
extern void circleinit(void);
extern void circle(void);

extern void allboff(void);
extern void allbon(void);

extern void boff1(void);
extern void boff2(void);
extern void boff3(void);
extern void boff4(void);
extern void boff5(void);
extern void boff6(void);
extern void boff7(void);


#define	CPC_MSG_LEN		93

char CPC_MY_IP[4] = {192,168,17,213} ; // vwpc10 192.168.17.212
int	cpflag = 1;

static resmgr_connect_funcs_t	connect_func;
static resmgr_io_funcs_t		io_func;
static iofunc_attr_t			attr;

void end_process (void) {
	// sleep(3);
}

/*
 * process_data: parse data which has been passed through the CPC pipeline, given
 *               a character buffer to copy to, an offset, and the number of bytes
 * @params: offset - offset within cpc pipeline (default to 0)
 *	        buffer - character array to which data should be copied
 *          nbytes - size of original command (used only for debugging)
 * @return: void
 */
void process_data (int offset, char *buffer, int nbytes) {
	int j;
	int ii, dd, cc;
	int debug = 1;
	unsigned char *cpc_msg;
	char cpc_target_ip[4];
	char cpc_com[16];
	char cpc_narg;
	long long cpc_arg_i[8];
	double cpc_arg_d[8];
	char cpc_arg_c[8][8];
	unsigned char cpc_arg_type[8];

	// if (debug) { for(j = 0; j < nbytes; j++) { printf("cpc_msg[%2d] = %d\n",j,(unsigned char)buffer[j]); }}

	// reset buffer, and copy entire CPC message to buffer
	cpc_msg = (unsigned char *)malloc(CPC_MSG_LEN);
	memset (cpc_msg, 0x00, sizeof(CPC_MSG_LEN));
	memcpy (cpc_msg, buffer, CPC_MSG_LEN);
	
	memcpy(cpc_target_ip, &cpc_msg[0],  4);
	if (strncmp(cpc_target_ip , CPC_MY_IP ,4) != 0) { return; }

	// print acknolwedgment of received command
	memcpy(cpc_com, &cpc_msg[4], 16);
	if (debug) { printf("CP > command : %s\n",cpc_com); }
	memcpy(&cpc_narg, &cpc_msg[20],  1);
	
	ii = 0, dd = 0, cc = 0;
	
	// copy arguments from CPC pipeline to buffer
	for (j = 0; j < cpc_narg; j++) {
	
		memcpy(&cpc_arg_type[j+1], &cpc_msg[21 + (9*j)], 1);
		if (cpc_arg_type[j+1] == 1) {
			memcpy(&cpc_arg_i[ii], &cpc_msg[22 + (9*j)], 8);
			ii++;
		}
		else if (cpc_arg_type[j+1] == 2) {
			memcpy(&cpc_arg_d[dd], &cpc_msg[22 + (9*j)], 8);
			dd++;
		}
		else if (cpc_arg_type[j+1] == 3) {
			memcpy(cpc_arg_c[cc], &cpc_msg[22 + (9*j)], 8);
			cc++;
		} // else memset(&cpc_msg[22+(9*j)], 0x00, 8);
	
	}

	// hardcode parsing of specific commands
	if (!(strncmp(cpc_com, "allzero", 7))) { allzero(); return; }
	if (!(strncmp(cpc_com, "allboff", 7))) { allboff(); return; }
	if (!(strncmp(cpc_com, "allbon" , 6))) { allbon(); return; }
	if (!(strncmp(cpc_com, "start"  , 5))) { pthread_create (NULL, NULL, (void *)start, NULL); }
	if (!(strncmp(cpc_com, "boff1"  , 5))) { boff1(); return; }
	if (!(strncmp(cpc_com, "boff2"  , 5))) { boff2(); return; }
	if (!(strncmp(cpc_com, "boff3"  , 5))) { boff3(); return; }
	if (!(strncmp(cpc_com, "boff4"  , 5))) { boff4(); return; }
	if (!(strncmp(cpc_com, "boff5"  , 5))) { boff5(); return; }
	if (!(strncmp(cpc_com, "boff6"  , 5))) { boff6(); return; }
	if (!(strncmp(cpc_com, "boff7"  , 5))) { boff7(); return; }   
	if (!(strncmp(cpc_com, "init"   , 4))) { init(); return; }
	if (!(strncmp(cpc_com, "sample" , 6))) { sample(); return; }
	if (!(strncmp(cpc_com, "lineinit", 8))) { lineinit(); return; }
	if (!(strncmp(cpc_com, "linedraw" , 8))) { line(); return; }
	if (!(strncmp(cpc_com, "circleinit" , 9))) { circleinit(); return; }
	if (!(strncmp(cpc_com, "circledraw" , 9))) { circle(); return; }
	if (!(strncmp(cpc_com, "set"    , 3))) { 
		if (dd != 6) { printf("ERR: <set> takes six double arguments\n"); return; }
		set_angles(cpc_arg_d);
		return;
	}
	if (!(strncmp(cpc_com, "endpoint", 8))) {
		if (dd != 6) { printf("ERR: <set> takes six double arguments\n"); return; }
		endpoint_moveto(cpc_arg_d);
		return;
	}
	if (!(strncmp(cpc_com, "fin"    , 3))) {
		fin();
		// sleep(1);
		// pthread_exit(NULL);*/
	}

}

/*
 * io_write: low-level function which writes data to data stream.
 *           tbh I have no idea what this is way too level for me.
 * @params: ctp - remsgr_context_t struct containing pipeline context
 *	        msg - io_write_t struct containing message to be written
 *          ocb - iofunc_ocb_t struct containing appropriate IO function
 * @return: error code
 */

int io_write (resmgr_context_t *ctp, io_write_t *msg, iofunc_ocb_t *ocb) {
	int	sts, nbytes, off, doffset, xtype;
	char *buffer;
	struct _xtype_offset *xoffset;
	
	sts = iofunc_write_verify (ctp, msg, ocb, NULL);
	if ( sts != EOK ) { return (sts); }
	
	// calculate appropriate offset for message
	xtype = msg->i.xtype & _IO_XTYPE_MASK;
	if ( xtype == _IO_XTYPE_MASK ) {
		xoffset = (struct _xtype_offset *)(&msg->i+1);
		doffset = sizeof(msg->i) + sizeof(*xoffset);
		off = xoffset->offset;
	} else if ( xtype == _IO_XTYPE_NONE ) {
		off = ocb->offset;
		doffset = sizeof(msg->i);
	} else { return (ENOSYS); }
	
	nbytes = msg->i.nbytes;
	if ((buffer = malloc(nbytes)) == NULL) { return (ENOSYS); }
	
	if (resmgr_msgread(ctp, buffer, nbytes, doffset) == -1) {
		free(buffer);
		return (errno);
	}
	
	process_data (off, buffer, nbytes);
	free (buffer);
	
	_IO_SET_WRITE_NBYTES (ctp, nbytes);
	
	if (nbytes) {
		ocb->attr->flags |= IOFUNC_ATTR_MTIME
						 |  IOFUNC_ATTR_DIRTY_TIME;
		if (xtype == _IO_XTYPE_NONE) {
			ocb->offset += nbytes;
		}
	}
	
	return (EOK);
}

/*
 * io-open: low-level function which opens the data stream
 * @params: ctp - remsgr_context_t struct containing pipeline context
 *	        msg - no idea what this does in this context
 *          handle - no idea what this does in this context
 *          ocb - iofunc_ocb_t struct containing appropriate IO function
 * @return: error code
 */

int io_open (resmgr_context_t *ctp, io_open_t *msg, RESMGR_HANDLE_T *handle, void *extra) {
	// static int open_flag;
	
	// if (!open_flag){
	// 	open_flag = 1;
		printf ("CP > accessed.\n");
		return (iofunc_open_default (ctp, msg, handle, extra));
	// } else {
	// 	printf ("CP > accessed by other client\n");
	// 	printf ("CP > access denied\n");
	// 	return (-2);
	// }
	
}

int io_close (resmgr_context_t *ctp, void *reserved, RESMGR_OCB_T *ocb) {
	printf ("CP > closed.\n");
	return (iofunc_close_ocb_default (ctp, reserved, ocb));
}

int io_read (resmgr_context_t *ctp, io_read_t *msg, iofunc_ocb_t *ocb) {
	printf ("CP > ERROR: CP does not support \"read.\"\n");
	return (iofunc_read_default (ctp, msg, ocb));
}

/*
 * cp: listen for opening of CPC pipeline in cpc.c, and attach context
 * @params: argc - number of arguments
 *          argv - argument list
 * @return: error code
 */
//thread_pool_t *
int cp (int argc, char **argv) {

	// thread_pool_attr_t	pool_attr;
	// thread_pool_t		*tpp;
	dispatch_t				*dpp;
	resmgr_attr_t			resmgr_attr;
	resmgr_context_t		*ctp;
	int						id;
	// int					cpc_id;
	
	// if ((cpc_id = open("/dev/cpc", O_WRONLY)) != -1){
	// 	printf ("/dev/cpc is already attached! \n");
	// 	close(cpc_id);
	// 	return (EXIT_FAILURE);
	// }	

	if ((dpp = dispatch_create() ) == NULL){
		fprintf (stderr,"%s: Unable to dispatch_create.\n",argv[0]);
		return (EXIT_FAILURE);
	}
	
	// memset (&pool_attr, 0, sizeof (pool_attr));
	
	// pool_attr.handle = dpp;
	// pool_attr.context_alloc = resmgr_context_alloc;
	// pool_attr.block_func = resmgr_block;
	// pool_attr.handler_func = resmgr_handler;
	// pool_attr.context_free = resmgr_context_free;
	
	// set up the number of threads that you want
	
	// pool_attr.lo_water = 0;
	// pool_attr.hi_water = 1;
	// pool_attr.increment = 1;
	// pool_attr.maximum = 10;

	// tpp = thread_pool_create (&pool_attr, POOL_FLAG_EXIT_SELF);

	// if (tpp == NULL){
	// 	fprintf (stderr,"%s: Unable to thread_pool_create.\n",argv[0]);
	// 	return (EXIT_FAILURE);
	// }
	
	iofunc_func_init (_RESMGR_CONNECT_NFUNCS, &connect_func, _RESMGR_IO_NFUNCS, &io_func);
	iofunc_attr_init (&attr, S_IFNAM | 0777, 0, 0);
	
	// override functions in "connect_func" and "io_func" as required
	// (ex) connect_func.open = io_open;
	// (ex) io_func.io_read = io_read;
	
	connect_func.open = io_open;
	io_func.read = io_read;
	io_func.write = io_write;
	io_func.close_ocb = io_close;
	
	memset (&resmgr_attr, 0, sizeof(resmgr_attr));
	resmgr_attr.nparts_max = 1;
	resmgr_attr.msg_max_size = 2048;
	
	id = resmgr_attach (dpp, &resmgr_attr, "/dev/cpc", _FTYPE_ANY, _RESMGR_FLAG_BEFORE,
						&connect_func, &io_func, &attr);
	
	if (id == -1) {
		fprintf (stderr,"%s: Unable to resmgr_attach.\n",argv[0]);
		return (EXIT_FAILURE);
	}
	
	ctp = resmgr_context_alloc (dpp);
	// thread_pool_start (tpp);
	
	while (cpflag) {
		if ((ctp = resmgr_block (ctp)) == NULL) {
			fprintf (stderr,"Unable to resmgr_block.\n");
			exit (EXIT_FAILURE);
		}
		resmgr_handler (ctp);
	}

	printf ("CP > cya!\n");
	return (EXIT_SUCCESS);
}
