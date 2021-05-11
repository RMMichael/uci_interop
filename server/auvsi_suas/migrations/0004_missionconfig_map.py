# Generated by Django 2.2.12 on 2020-05-25 22:53

import django.core.validators
from django.db import migrations, models
import django.db.models.deletion


class Migration(migrations.Migration):

    dependencies = [
        ('auvsi_suas', '0003_static_params'),
    ]

    operations = [
        migrations.AddField(
            model_name='missionconfig',
            name='map_center_pos',
            field=models.ForeignKey(
                default=1,
                on_delete=django.db.models.deletion.CASCADE,
                related_name='missionconfig_map_center_pos',
                to='auvsi_suas.GpsPosition'),
            preserve_default=False,
        ),
        migrations.AddField(
            model_name='missionconfig',
            name='map_height_ft',
            field=models.FloatField(
                default=1,
                validators=[django.core.validators.MinValueValidator(1)]),
            preserve_default=False,
        ),
    ]
