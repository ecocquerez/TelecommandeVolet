#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=cof
DEBUGGABLE_SUFFIX=cof
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/TelecommandeVolet.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=cof
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/TelecommandeVolet.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/MSPI.o ${OBJECTDIR}/SymbolTime.o ${OBJECTDIR}/_ext/1336284664/P2P.o ${OBJECTDIR}/MRF24J40.o ${OBJECTDIR}/TelecommandeVolet.o
POSSIBLE_DEPFILES=${OBJECTDIR}/MSPI.o.d ${OBJECTDIR}/SymbolTime.o.d ${OBJECTDIR}/_ext/1336284664/P2P.o.d ${OBJECTDIR}/MRF24J40.o.d ${OBJECTDIR}/TelecommandeVolet.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/MSPI.o ${OBJECTDIR}/SymbolTime.o ${OBJECTDIR}/_ext/1336284664/P2P.o ${OBJECTDIR}/MRF24J40.o ${OBJECTDIR}/TelecommandeVolet.o


CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
	${MAKE} ${MAKE_OPTIONS} -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/TelecommandeVolet.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=18F25K50
MP_PROCESSOR_OPTION_LD=18f25k50
MP_LINKER_DEBUG_OPTION=
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/MSPI.o: MSPI.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/MSPI.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -p$(MP_PROCESSOR_OPTION) -I"." -I".." -I"../.." -ms -oa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/MSPI.o   MSPI.c 
	@${DEP_GEN} -d ${OBJECTDIR}/MSPI.o 
	@${FIXDEPS} "${OBJECTDIR}/MSPI.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/SymbolTime.o: SymbolTime.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/SymbolTime.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -p$(MP_PROCESSOR_OPTION) -I"." -I".." -I"../.." -ms -oa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/SymbolTime.o   SymbolTime.c 
	@${DEP_GEN} -d ${OBJECTDIR}/SymbolTime.o 
	@${FIXDEPS} "${OBJECTDIR}/SymbolTime.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/_ext/1336284664/P2P.o: ../RecepteurVolet/P2P.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1336284664 
	@${RM} ${OBJECTDIR}/_ext/1336284664/P2P.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -p$(MP_PROCESSOR_OPTION) -I"." -I".." -I"../.." -ms -oa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/_ext/1336284664/P2P.o   ../RecepteurVolet/P2P.c 
	@${DEP_GEN} -d ${OBJECTDIR}/_ext/1336284664/P2P.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1336284664/P2P.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/MRF24J40.o: MRF24J40.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/MRF24J40.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -p$(MP_PROCESSOR_OPTION) -I"." -I".." -I"../.." -ms -oa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/MRF24J40.o   MRF24J40.c 
	@${DEP_GEN} -d ${OBJECTDIR}/MRF24J40.o 
	@${FIXDEPS} "${OBJECTDIR}/MRF24J40.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/TelecommandeVolet.o: TelecommandeVolet.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/TelecommandeVolet.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -p$(MP_PROCESSOR_OPTION) -I"." -I".." -I"../.." -ms -oa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/TelecommandeVolet.o   TelecommandeVolet.c 
	@${DEP_GEN} -d ${OBJECTDIR}/TelecommandeVolet.o 
	@${FIXDEPS} "${OBJECTDIR}/TelecommandeVolet.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
else
${OBJECTDIR}/MSPI.o: MSPI.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/MSPI.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -I"." -I".." -I"../.." -ms -oa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/MSPI.o   MSPI.c 
	@${DEP_GEN} -d ${OBJECTDIR}/MSPI.o 
	@${FIXDEPS} "${OBJECTDIR}/MSPI.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/SymbolTime.o: SymbolTime.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/SymbolTime.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -I"." -I".." -I"../.." -ms -oa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/SymbolTime.o   SymbolTime.c 
	@${DEP_GEN} -d ${OBJECTDIR}/SymbolTime.o 
	@${FIXDEPS} "${OBJECTDIR}/SymbolTime.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/_ext/1336284664/P2P.o: ../RecepteurVolet/P2P.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1336284664 
	@${RM} ${OBJECTDIR}/_ext/1336284664/P2P.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -I"." -I".." -I"../.." -ms -oa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/_ext/1336284664/P2P.o   ../RecepteurVolet/P2P.c 
	@${DEP_GEN} -d ${OBJECTDIR}/_ext/1336284664/P2P.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1336284664/P2P.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/MRF24J40.o: MRF24J40.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/MRF24J40.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -I"." -I".." -I"../.." -ms -oa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/MRF24J40.o   MRF24J40.c 
	@${DEP_GEN} -d ${OBJECTDIR}/MRF24J40.o 
	@${FIXDEPS} "${OBJECTDIR}/MRF24J40.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/TelecommandeVolet.o: TelecommandeVolet.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/TelecommandeVolet.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -I"." -I".." -I"../.." -ms -oa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/TelecommandeVolet.o   TelecommandeVolet.c 
	@${DEP_GEN} -d ${OBJECTDIR}/TelecommandeVolet.o 
	@${FIXDEPS} "${OBJECTDIR}/TelecommandeVolet.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/TelecommandeVolet.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk  Libraries/clib.lib Libraries/p18f25k50.lib  Linker/specif18f25k50_g.lkr
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_LD} $(MP_EXTRA_LD_PRE) "Linker\specif18f25k50_g.lkr"  -p$(MP_PROCESSOR_OPTION_LD)  -w -x -u_DEBUG -l"E:/Utilisateurs/Eric/Documents/GitHub/TelecommandeVolet/Libraries"  -z__MPLAB_BUILD=1  -u_CRUNTIME -z__MPLAB_DEBUG=1 -z__MPLAB_DEBUGGER_PK3=1 $(MP_LINKER_DEBUG_OPTION) -l ${MP_CC_DIR}\\..\\lib  -o dist/${CND_CONF}/${IMAGE_TYPE}/TelecommandeVolet.${IMAGE_TYPE}.${OUTPUT_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}  Libraries\clib.lib Libraries\p18f25k50.lib 
else
dist/${CND_CONF}/${IMAGE_TYPE}/TelecommandeVolet.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk  Libraries/clib.lib Libraries/p18f25k50.lib Linker/specif18f25k50_g.lkr
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_LD} $(MP_EXTRA_LD_PRE) "Linker\specif18f25k50_g.lkr"  -p$(MP_PROCESSOR_OPTION_LD)  -w  -l"E:/Utilisateurs/Eric/Documents/GitHub/TelecommandeVolet/Libraries"  -z__MPLAB_BUILD=1  -u_CRUNTIME -l ${MP_CC_DIR}\\..\\lib  -o dist/${CND_CONF}/${IMAGE_TYPE}/TelecommandeVolet.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}  Libraries\clib.lib Libraries\p18f25k50.lib 
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/default
	${RM} -r dist/default

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
