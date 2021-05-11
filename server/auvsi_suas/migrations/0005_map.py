# Generated by Django 2.2.17 on 2021-01-19 13:19

from django.conf import settings
from django.db import migrations, models
import django.db.models.deletion


class Migration(migrations.Migration):

    dependencies = [
        migrations.swappable_dependency(settings.AUTH_USER_MODEL),
        ('auvsi_suas', '0004_missionconfig_map'),
    ]

    operations = [
        migrations.CreateModel(
            name='Map',
            fields=[
                ('id',
                 models.AutoField(auto_created=True,
                                  primary_key=True,
                                  serialize=False,
                                  verbose_name='ID')),
                ('uploaded_map', models.ImageField(blank=True,
                                                   upload_to='maps')),
                ('quality',
                 models.IntegerField(blank=True,
                                     choices=[(0, 'INSUFFICIENT'),
                                              (1, 'MEDIUM'), (2, 'HIGH')],
                                     null=True)),
                ('mission',
                 models.ForeignKey(on_delete=django.db.models.deletion.CASCADE,
                                   to='auvsi_suas.MissionConfig')),
                ('user',
                 models.ForeignKey(on_delete=django.db.models.deletion.CASCADE,
                                   to=settings.AUTH_USER_MODEL)),
            ],
        ),
    ]