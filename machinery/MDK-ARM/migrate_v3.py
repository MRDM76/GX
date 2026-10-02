from pathlib import Path
import shutil, datetime
src = Path(r'C:\Users\62575\Desktop\GX (2)\GX\machinery')
dst = Path(__file__).resolve().parent
backup = dst / 'build' / ('before-v3-' + datetime.datetime.now().strftime('%Y%m%d-%H%M%S'))
backup.mkdir(parents=True)
for name in ['Motor', '.eide', 'machinery.uvprojx', '工作日志.md']:
    p = dst/name
    if p.is_dir(): shutil.copytree(p, backup/name)
    elif p.exists(): shutil.copy2(p, backup/name)
shutil.copytree(dst.parent/'Core', backup/'Core')
shutil.copy2(dst.parent/'machinery.ioc', backup/'machinery.ioc')
shutil.copytree(src/'Core', dst.parent/'Core', dirs_exist_ok=True)
shutil.copy2(src/'machinery.ioc', dst.parent/'machinery.ioc')
shutil.copytree(src/'MDK-ARM'/'Motor', dst/'Motor', dirs_exist_ok=True)
for p in (src/'MDK-ARM').glob('*.md'):
    shutil.copy2(p, dst/p.name)
for p in (dst/'Motor').iterdir():
    if p.suffix in ['.c','.h']:
        target = dst.parent/'Core'/('Src' if p.suffix=='.c' else 'Inc')/p.name
        shutil.copy2(p, target)
        p.unlink()
names = ['motor', 'pid', 'motor_command', 'motor_uart']
p=dst/'.eide/eide.yml'; s=p.read_text(encoding='utf-8')
for n in names: s=s.replace('Motor/'+n+'.c', '../Core/Src/'+n+'.c')
for n in names[2:]:
    if '../Core/Src/'+n+'.c' not in s: s=s.replace('                - path: ../Core/Src/pid.c', '                - path: ../Core/Src/pid.c\n                - path: ../Core/Src/'+n+'.c')
s=s.replace('        - Motor\n','')
hal='D:/Environment/STM32CubeMX/Repository/STM32Cube_FW_F1_V1.8.7/Drivers/STM32F1xx_HAL_Driver/Src'
if 'stm32f1xx_hal_uart.c' not in s: s=s.replace('    - name: Drivers', '    - name: Drivers',1).replace('            - path: '+hal+'/stm32f1xx_hal.c','            - path: '+hal+'/stm32f1xx_hal_uart.c\n            - path: '+hal+'/stm32f1xx_hal.c')
p.write_text(s,encoding='utf-8')
p=dst/'machinery.uvprojx';s=p.read_text(encoding='utf-8');s=s.replace(';./Motor','')
for n in names: s=s.replace('./Motor/'+n+'.c','../Core/Src/'+n+'.c')
extra=''
for n in names[2:]: extra+=f'<File><FileName>{n}.c</FileName><FileType>1</FileType><FilePath>../Core/Src/{n}.c</FilePath></File>\n'
extra+=f'<File><FileName>stm32f1xx_hal_uart.c</FileName><FileType>1</FileType><FilePath>{hal}/stm32f1xx_hal_uart.c</FilePath></File>\n'
idx=s.index('<Files>',s.index('<GroupName>Motor</GroupName>'))+len('<Files>');s=s[:idx]+extra+s[idx:];p.write_text(s,encoding='utf-8')
p=dst.parent/'Core/Inc/stm32f1xx_hal_conf.h';s=p.read_text();s=s.replace('/*#define HAL_UART_MODULE_ENABLED   */','#define HAL_UART_MODULE_ENABLED');p.write_text(s)
p=dst/'Motor/build_check.ps1';s=p.read_text();s=s.replace("$sources = @($params.sourceList)","$sources = @($params.sourceList | Where-Object { $_ -notmatch 'Motor[/\\\\]' })")
s=s.replace("'Motor/motor.c', 'Motor/pid.c'", "'../Core/Src/motor.c', '../Core/Src/pid.c', '../Core/Src/motor_command.c', '../Core/Src/motor_uart.c', \"$hal/stm32f1xx_hal_uart.c\"")
s=s.replace("@('Motor')","@('../Core/Inc')");p.write_text(s)
p=dst/'Motor/tests/run_msvc.cmd';s=p.read_text();s=s.replace('/I Motor /','/I ../Core/Inc /')
for n in names: s=s.replace('Motor/'+n+'.c','../Core/Src/'+n+'.c')
p.write_text(s)
print('Backup:', backup)
