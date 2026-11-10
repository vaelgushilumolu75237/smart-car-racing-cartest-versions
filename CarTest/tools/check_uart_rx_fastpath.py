"""Check the emitted ARM IRQ, not a source-text promise of inlining.

This is a code-generation check; it does not measure board execution time.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--elf',type=Path,required=True)
    parser.add_argument('--objdump',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    result=subprocess.run([str(args.objdump),'-d','--disassemble=USART1_IRQHandler',str(args.elf)],
                          check=True,capture_output=True,text=True)
    text=result.stdout
    instructions=[line for line in text.splitlines() if re.match(r'^\s*[0-9a-f]+:\s',line) and '.word' not in line]
    calls=re.findall(r'\bbl(?:\.w)?\s+[0-9a-f]+\s+<([^>]+)>',text)
    allowed={'HAL_GetTick','uart_service_on_abort','uart_service_on_error','car_control_note_fast_stop_isr'}
    if not instructions or 'USART1_IRQHandler>:' not in text:raise RuntimeError('Missing IRQ code')
    if re.search(r'\bblx\b',text) or set(calls)-allowed:
        raise RuntimeError('RX IRQ contains an indirect or unexpected call: '+repr(calls))
    if 'HAL_GetTick' not in calls or 'uart_service_on_error' not in calls or 'uart_service_on_abort' not in calls:
        raise RuntimeError('IRQ timestamp/error/overflow path was lost')
    if len(re.findall(r'\bdmb\b',text))<2 or not re.search(r'\bPRIMASK\b',text):
        raise RuntimeError('RX publication or IRQ-mask discipline was lost')
    if not re.search(r'\bcmp(?:\.w)?\s+[^\n]*#16\b',text):
        raise RuntimeError('FIFO-sized 16-byte IRQ limit was lost')
    report={'passed':True,'elf_sha256':hashlib.sha256(args.elf.read_bytes()).hexdigest(),
            'irq_instructions_static':len(instructions),'calls':calls,
            'indirect_calls':False,'publication_barriers':True,'bounded_batch':16,
            'board_timing_measured':False,'hardware_actions':'none'}
    args.output.write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report))


if __name__=='__main__':main()
