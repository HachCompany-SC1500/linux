#include <linux/bug.h>
#include <linux/io.h>
#include <linux/types.h>
#include <linux/kdebug.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/uaccess.h>
#include <linux/hardirq.h>
#include <asm/unwinder.h>
#include <asm/system.h>
#include <asm/dma.h>
#include <asm/dma-sh.h>


#ifdef CONFIG_HLIRQ_SUPPORT
extern void hl_irq_nmi_handler(void);
#endif

#ifdef CONFIG_RL78_SUPPORT
extern void rl78_irq_nmi_handler(void);
#endif

#ifdef CONFIG_GENERIC_BUG
static void handle_BUG(struct pt_regs *regs)
{
	const struct bug_entry *bug;
	unsigned long bugaddr = regs->pc;
	enum bug_trap_type tt;

	if (!is_valid_bugaddr(bugaddr))
		goto invalid;

	bug = find_bug(bugaddr);

	/* Switch unwinders when unwind_stack() is called */
	if (bug->flags & BUGFLAG_UNWINDER)
		unwinder_faulted = 1;

	tt = report_bug(bugaddr, regs);
	if (tt == BUG_TRAP_TYPE_WARN) {
		regs->pc += instruction_size(bugaddr);
		return;
	}

invalid:
	die("Kernel BUG", regs, TRAPA_BUG_OPCODE & 0xff);
}

int is_valid_bugaddr(unsigned long addr)
{
	insn_size_t opcode;

	if (addr < PAGE_OFFSET)
		return 0;
	if (probe_kernel_address((insn_size_t *)addr, opcode))
		return 0;
	if (opcode == TRAPA_BUG_OPCODE)
		return 1;

	return 0;
}
#endif

/*
 * Generic trap handler.
 */
BUILD_TRAP_HANDLER(debug)
{
	TRAP_HANDLER_DECL;

	/* Rewind */
	regs->pc -= instruction_size(__raw_readw(regs->pc - 4));

	if (notify_die(DIE_TRAP, "debug trap", regs, 0, vec & 0xff,
		       SIGTRAP) == NOTIFY_STOP)
		return;

	force_sig(SIGTRAP, current);
}

/*
 * Special handler for BUG() traps.
 */
BUILD_TRAP_HANDLER(bug)
{
	TRAP_HANDLER_DECL;

	/* Rewind */
	regs->pc -= instruction_size(__raw_readw(regs->pc - 4));

	if (notify_die(DIE_TRAP, "bug trap", regs, 0, TRAPA_BUG_OPCODE & 0xff,
		       SIGTRAP) == NOTIFY_STOP)
		return;

#ifdef CONFIG_GENERIC_BUG
	if (__kernel_text_address(instruction_pointer(regs))) {
		insn_size_t insn = *(insn_size_t *)instruction_pointer(regs);
		if (insn == TRAPA_BUG_OPCODE)
			handle_BUG(regs);
		return;
	}
#endif

	force_sig(SIGTRAP, current);
}

BUILD_TRAP_HANDLER(nmi)
{
	unsigned long dmaor;
	unsigned int cpu = smp_processor_id();
	TRAP_HANDLER_DECL;

	nmi_enter();
	nmi_count(cpu)++;


	switch (notify_die(DIE_NMI, "NMI", regs, 0, vec & 0xff, SIGINT)) {
	case NOTIFY_OK:
	case NOTIFY_STOP:
		break;
	case NOTIFY_BAD:
		die("Fatal Non-Maskable Interrupt", regs, SIGINT);
	default:
          // printk(KERN_ALERT "Got NMI, wake HACH-LANGE application...\n");
#ifdef CONFIG_HLIRQ_SUPPORT
		hl_irq_nmi_handler();
#endif
#ifdef CONFIG_RL78_SUPPORT
		rl78_irq_nmi_handler();
#endif
        break;
	}

/* every NMI usually stops all active DMA transfers. These lines simply reanimate the */
/* DMA channels so that the transfers are resumed                                     */
#if defined(CONFIG_SH_HICO7723) || defined(CONFIG_SH_HICO7724)
	dmaor = __raw_readw(SH_DMAC_BASE0 + DMAOR);
	dmaor &= ~(DMAOR_NMIF | DMAOR_AE); // resetting NMI flag and address error flag
	__raw_writew( dmaor, SH_DMAC_BASE0 + DMAOR );
	dmaor |= DMAOR_INIT;               // restarting DMA
	__raw_writew( dmaor, SH_DMAC_BASE0 + DMAOR );

	dmaor = __raw_readw(SH_DMAC_BASE1 + DMAOR);
	dmaor &= ~(DMAOR_NMIF | DMAOR_AE); // resetting NMI flag and address error flag
	__raw_writew( dmaor, SH_DMAC_BASE1 + DMAOR );
	dmaor |= DMAOR_INIT;               // restarting DMA
	__raw_writew( dmaor, SH_DMAC_BASE1 + DMAOR );
#endif

	nmi_exit();
}
