#پیاده سازی کنترلکننده به منظور حرکت ربات از یک نقطه به نقطه دیگر در یک راستای دیگر
یک کلاس برای کنترل کننده پی آی دی و چند تابع برای اعمال سرعت به موتورها تعریف می شود. 
#الگوریتم مورد استفاده در حل مسئله
ابتدا زاویه ربات تغیر داده می شود تا روبروی نقطه مقصد قرار بگیرد. سپس به آن نقطه میرود.سپس زاویه ربات به زاویه دلخواه تغییر داده میشود.
#بررسی اثر هر یک از کنترل کننده ها در مجموعه
در کنترل کننده های PIوP حرکت یکنواخت ولی در PDوPID حرکت غیر یکنواخت است. در پایان حرکت به علت ناحیه مرده، خطا صفر نمیشود و حرکت نوسانی کوچک دارد.
#روش گسسته سازی
کنترلر ها به روش بکوارد گسسته شده اند زیرا پایداری حفظ میشود.
#بررسی اثر ناحیه ی مرده موتورها و تاثیرآن در مجموعه
زمانی که  اندازه سرعت اعمالی به موتور ها کمتر از 0.15 باشد ، سرعت صفر میشود. این باعث میشود تا خطا صفر نشود.