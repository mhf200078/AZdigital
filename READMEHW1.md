#پیاده سازی کنترلکننده به منظور حرکت ربات از یک نقطه به نقطه دیگر در یک راستا
یک کلاس برای کنترل کننده پی آی دی و چند تابع برای اعمال سرعت به موتورها تعریف می شود. با صفر کردن خطا ربات به نقطه مورد نظر میرسد.
#الگوریتم مورد استفاده در حل مسئله
تفاضل موقعیت ربات و موقعیت دلخواه به کنترلر داده میشود و با اعمال سرعت به موتور ها این تفاضل کاهش میابد.
#بررسی اثر هر یک از کنترل کننده ها در مجموعه
در کنترل کننده های PIوP حرکت یکنواخت ولی در PDوPID حرکت غیر یکنواخت است. در پایان حرکت به علت ناحیه مرده، خطا صفر نمیشود و حرکت نوسانی کوچک دارد.
#روش گسسته سازی
کنترلر ها به روش بکوارد گسسته شده اند زیرا پایداری حفظ میشود.
#بررسی اثر ناحیه ی مرده موتورها و تاثیرآن در مجموعه
زمانی که  اندازه سرعت اعمالی به موتور ها کمتر از 0.15 باشد ، سرعت صفر میشود. این باعث میشود تا خطا صفر نشود.
