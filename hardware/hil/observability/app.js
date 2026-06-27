document.addEventListener('DOMContentLoaded', () => {
    let cteChart = null;
    let steeringChart = null;
    let liveInterval = null;

    // Hook primary refresh button
    document.getElementById('btn-refresh').addEventListener('click', () => {
        refreshDashboard(false);
    });

    // Hook Live checkbox
    const chkLive = document.getElementById('chk-live');
    chkLive.addEventListener('change', (e) => {
        if (e.target.checked) {
            // Instantly sync once, then start interval
            refreshDashboard(true);
            liveInterval = setInterval(() => {
                refreshDashboard(true);
            }, 500);
        } else {
            if (liveInterval) {
                clearInterval(liveInterval);
                liveInterval = null;
            }
        }
    });

    // Initial load
    refreshDashboard(false);

    function refreshDashboard(isSilent = false) {
        if (!isSilent) {
            setLoadingState(true);
        }

        Promise.all([
            fetch('/api/telemetry').then(r => r.json()),
            fetch('/api/summary').then(r => r.json()),
            fetch('/api/diagnosis').then(r => r.json())
        ])
        .then(([telemetry, summary, diagnosis]) => {
            if (telemetry.error || summary.error || diagnosis.error) {
                const errMsg = telemetry.error || summary.error || diagnosis.error;
                if (!isSilent) {
                    alert(errMsg);
                } else {
                    console.log('Observability Sync Error:', errMsg);
                }
                return;
            }

            // 1. Update statistics cards
            document.getElementById('val-rmse').innerText = `${summary.rmse_cte_px.toFixed(2)} px`;
            document.getElementById('val-max-cte').innerText = `${summary.max_abs_cte_px.toFixed(2)} px`;
            document.getElementById('val-safety-margin').innerText = `${summary.min_safety_margin_h.toFixed(3)}`;
            document.getElementById('val-chattering').innerText = `${summary.steering_chattering.toFixed(2)}`;

            updateCardAesthetics(summary);

            // 2. Load separate telemetry streams
            const unstableData = telemetry.unstable || [];
            const stableData = telemetry.stable || [];

            // Relative timestamps starting at 0 for both subsets
            const unstableStart = unstableData.length > 0 ? unstableData[0].time : 0;
            const stableStart = stableData.length > 0 ? stableData[0].time : 0;

            const uTime = unstableData.map(d => d.time - unstableStart);
            const uCte = unstableData.map(d => d.cte);
            const uSteering = unstableData.map(d => d.steering);
            const uH = unstableData.map(d => d.h);

            const sTime = stableData.map(d => d.time - stableStart);
            const sCte = stableData.map(d => d.cte);
            const sSteering = stableData.map(d => d.steering);
            const sH = stableData.map(d => d.h);

            // 3. Update Table fields (Behavior-based Classification)
            updateDiagnosticsTable(unstableData, stableData);

            // 4. Render AutoDiag Findings & Score
            updateDiagnosisUI(diagnosis);

            // 5. Render or update Chart 1 (CTE Tracking)
            renderCteChart(uTime, uCte, sTime, sCte);

            // 6. Render or update Chart 2 (Steering & Safety h)
            renderSteeringChart(uTime, uSteering, uH, sTime, sSteering, sH);
        })
        .catch(err => {
            console.error('API Error:', err);
        })
        .finally(() => {
            if (!isSilent) {
                setLoadingState(false);
            }
        });
    }

    function setLoadingState(isLoading) {
        const btn = document.getElementById('btn-refresh');
        if (isLoading) {
            btn.classList.add('loading');
            btn.innerHTML = `Syncing...`;
        } else {
            btn.classList.remove('loading');
            btn.innerHTML = `
                <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M23 4v6h-6M1 20v-6h6M3.51 9a9 9 0 0 1 14.85-3.36L23 10M1 14l4.64 4.36A9 9 0 0 0 20.49 15"/></svg>
                Sync Telemetry
            `;
        }
    }

    function updateCardAesthetics(summary) {
        const marginCard = document.getElementById('card-safety-margin');
        if (summary.min_safety_margin_h < 0.0) {
            marginCard.className = 'card stat-card red-glowing';
        } else if (summary.min_safety_margin_h < 0.1) {
            marginCard.className = 'card stat-card orange-glowing';
        } else {
            marginCard.className = 'card stat-card cyan-glowing';
        }
    }

    function updateDiagnosticsTable(unstableData, stableData) {
        if (unstableData.length > 0) {
            const uAbsCte = unstableData.map(d => Math.abs(d.cte));
            const uMeanCte = uAbsCte.reduce((a,b)=>a+b, 0) / uAbsCte.length;
            
            const uAbsHe = unstableData.map(d => Math.abs(d.he * (180 / Math.PI)));
            const uMeanHe = uAbsHe.reduce((a,b)=>a+b, 0) / uAbsHe.length;
            
            const minH = Math.min(...unstableData.map(d => d.h));

            document.getElementById('tbl-u-cte').innerText = `${uMeanCte.toFixed(1)} px`;
            document.getElementById('tbl-u-he').innerText = `${uMeanHe.toFixed(1)} deg`;
            document.getElementById('tbl-u-safety').innerHTML = minH < 0.0 
                ? `<span class="text-danger">Safety Violation (min h=${minH.toFixed(2)})</span>` 
                : `<span class="text-success">Clear (min h=${minH.toFixed(2)})</span>`;
            
            const uSats = unstableData.filter(d => Math.abs(d.steering) >= 1.29).length;
            const uSatRatio = (uSats / unstableData.length) * 100;
            document.getElementById('tbl-u-sat').innerHTML = uSatRatio > 15
                ? `<span class="text-danger">Saturated (${uSatRatio.toFixed(0)}% of run)</span>`
                : `<span>Smooth (${uSatRatio.toFixed(0)}%)</span>`;
        }

        if (stableData.length > 0) {
            const sAbsCte = stableData.map(d => Math.abs(d.cte));
            const sMeanCte = sAbsCte.reduce((a,b)=>a+b, 0) / sAbsCte.length;
            
            const sAbsHe = stableData.map(d => Math.abs(d.he * (180 / Math.PI)));
            const sMeanHe = sAbsHe.reduce((a,b)=>a+b, 0) / sAbsHe.length;
            
            const minH = Math.min(...stableData.map(d => d.h));

            document.getElementById('tbl-s-cte').innerText = `${sMeanCte.toFixed(1)} px`;
            document.getElementById('tbl-s-he').innerText = `${sMeanHe.toFixed(1)} deg`;
            document.getElementById('tbl-s-safety').innerHTML = minH < 0.0 
                ? `<span class="text-danger">Safety Violation (min h=${minH.toFixed(2)})</span>` 
                : `<span class="text-success">Clear (min h=${minH.toFixed(2)})</span>`;

            const sSats = stableData.filter(d => Math.abs(d.steering) >= 1.29).length;
            const sSatRatio = (sSats / stableData.length) * 100;
            document.getElementById('tbl-s-sat').innerHTML = sSatRatio > 15
                ? `<span class="text-danger">Saturated (${sSatRatio.toFixed(0)}%)</span>`
                : `<span class="text-success">Smooth (${sSatRatio.toFixed(0)}%)</span>`;
        }
    }

    function updateDiagnosisUI(diagnosis) {
        // 1. Update Controller Score
        const scoreVal = document.getElementById('val-score');
        scoreVal.innerText = `${diagnosis.score.toFixed(0)} / 100`;
        
        if (diagnosis.score >= 80) {
            scoreVal.style.color = 'var(--accent-green)';
        } else if (diagnosis.score >= 50) {
            scoreVal.style.color = 'var(--accent-orange)';
        } else {
            scoreVal.style.color = 'var(--accent-red)';
        }

        // 2. Render diagnosis cards
        const listContainer = document.getElementById('diagnoses-list');
        listContainer.innerHTML = '';

        if (!diagnosis.diagnoses || diagnosis.diagnoses.length === 0) {
            listContainer.innerHTML = `
                <div class="diagnosis-item-placeholder" style="color: var(--accent-green); font-style: italic; font-size: 0.9rem;">
                    ✔ No issues detected. Controller operates fully within normal thresholds.
                </div>
            `;
            return;
        }

        diagnosis.diagnoses.forEach(diag => {
            const item = document.createElement('div');
            item.className = 'diagnosis-item';

            let badgeClass = 'badge-ok';
            if (diag.severity === 'ERROR') {
                badgeClass = 'badge-error';
            } else if (diag.severity === 'WARN') {
                badgeClass = 'badge-warn';
            }

            const evidenceHtml = diag.evidence.map(e => `<li>${e}</li>`).join('');
            const causesHtml = diag.likely_causes.map(c => `<li>${c}</li>`).join('');
            const fixesHtml = diag.recommended_fixes.map(f => `<li>${f}</li>`).join('');

            item.innerHTML = `
                <div class="diagnosis-header-row">
                    <div class="diagnosis-title-area">
                        <span class="diagnosis-badge ${badgeClass}">${diag.severity}</span>
                        <h3 class="diagnosis-title">${diag.issue}</h3>
                    </div>
                </div>
                <div class="diagnosis-details-grid">
                    <div class="detail-col evidence-col">
                        <h4>Telemetry Evidence</h4>
                        <ul>${evidenceHtml}</ul>
                    </div>
                    <div class="detail-col causes-col">
                        <h4>Probable Root Causes</h4>
                        <ul>${causesHtml}</ul>
                    </div>
                    <div class="detail-col fixes-col">
                        <h4>Recommended Action Items</h4>
                        <ul>${fixesHtml}</ul>
                    </div>
                </div>
            `;
            listContainer.appendChild(item);
        });
    }

    function renderCteChart(uTime, uCte, sTime, sCte) {
        const ctx = document.getElementById('chart-cte').getContext('2d');
        if (cteChart) {
            cteChart.data.datasets[0].data = uCte.map((c, i) => ({ x: uTime[i], y: c }));
            cteChart.data.datasets[1].data = sCte.map((c, i) => ({ x: sTime[i], y: c }));
            cteChart.update('none'); // Update without animation for performance in live mode
            return;
        }

        cteChart = new Chart(ctx, {
            type: 'line',
            data: {
                datasets: [
                    {
                        label: 'Unstable Trial Profile',
                        data: uCte.map((c, i) => ({ x: uTime[i], y: c })),
                        borderColor: '#ff3355',
                        borderWidth: 2,
                        pointRadius: 0,
                        tension: 0.1
                    },
                    {
                        label: 'Stable Trial Profile',
                        data: sCte.map((c, i) => ({ x: sTime[i], y: c })),
                        borderColor: '#33ff99',
                        borderWidth: 2.5,
                        pointRadius: 0,
                        tension: 0.1
                    }
                ]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                animation: false,
                scales: {
                    x: {
                        type: 'linear',
                        title: { display: true, text: 'Time (s)', color: '#9090b0' },
                        grid: { color: 'rgba(255, 255, 255, 0.05)' },
                        ticks: { color: '#9090b0' }
                    },
                    y: {
                        title: { display: true, text: 'Cross-Track Error (px)', color: '#9090b0' },
                        grid: { color: 'rgba(255, 255, 255, 0.05)' },
                        ticks: { color: '#9090b0' },
                        suggestedMin: -35,
                        suggestedMax: 35
                    }
                },
                plugins: {
                    legend: {
                        labels: { color: '#f0f0f5', font: { family: 'Outfit' } }
                    }
                }
            }
        });
    }

    function renderSteeringChart(uTime, uSteering, uH, sTime, sSteering, sH) {
        const ctx = document.getElementById('chart-steering').getContext('2d');
        if (steeringChart) {
            steeringChart.data.datasets[0].data = uSteering.map((s, i) => ({ x: uTime[i], y: s }));
            steeringChart.data.datasets[1].data = uH.map((h, i) => ({ x: uTime[i], y: h }));
            steeringChart.data.datasets[2].data = sSteering.map((s, i) => ({ x: sTime[i], y: s }));
            steeringChart.data.datasets[3].data = sH.map((h, i) => ({ x: sTime[i], y: h }));
            steeringChart.update('none'); // Update without animation for performance in live mode
            return;
        }

        steeringChart = new Chart(ctx, {
            type: 'line',
            data: {
                datasets: [
                    {
                        label: 'Unstable Steering Command (rad)',
                        data: uSteering.map((s, i) => ({ x: uTime[i], y: s })),
                        borderColor: '#ff3355',
                        borderWidth: 1.5,
                        borderDash: [4, 4],
                        pointRadius: 0,
                        yAxisID: 'y'
                    },
                    {
                        label: 'Unstable Barrier h(x)',
                        data: uH.map((h, i) => ({ x: uTime[i], y: h })),
                        borderColor: '#ff9933',
                        borderWidth: 1.5,
                        pointRadius: 0,
                        yAxisID: 'y1'
                    },
                    {
                        label: 'Stable Steering Command (rad)',
                        data: sSteering.map((s, i) => ({ x: sTime[i], y: s })),
                        borderColor: '#33ff99',
                        borderWidth: 2,
                        pointRadius: 0,
                        yAxisID: 'y'
                    },
                    {
                        label: 'Stable Barrier h(x)',
                        data: sH.map((h, i) => ({ x: sTime[i], y: h })),
                        borderColor: '#33ccff',
                        borderWidth: 2,
                        pointRadius: 0,
                        yAxisID: 'y1'
                    }
                ]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                animation: false,
                scales: {
                    x: {
                        type: 'linear',
                        title: { display: true, text: 'Time (s)', color: '#9090b0' },
                        grid: { color: 'rgba(255, 255, 255, 0.05)' },
                        ticks: { color: '#9090b0' }
                    },
                    y: {
                        type: 'linear',
                        position: 'left',
                        title: { display: true, text: 'Steering Input (rad)', color: '#9090b0' },
                        grid: { color: 'rgba(255, 255, 255, 0.05)' },
                        ticks: { color: '#9090b0' },
                        suggestedMin: -1.35,
                        suggestedMax: 1.35
                    },
                    y1: {
                        type: 'linear',
                        position: 'right',
                        title: { display: true, text: 'Barrier Clearance h(x)', color: '#9090b0' },
                        grid: { drawOnChartArea: false },
                        ticks: { color: '#9090b0' },
                        suggestedMin: -0.5,
                        suggestedMax: 5.0
                    }
                },
                plugins: {
                    legend: {
                        labels: { color: '#f0f0f5', font: { family: 'Outfit' } }
                    }
                }
            }
        });
    }
});
