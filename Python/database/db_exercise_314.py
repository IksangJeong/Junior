"""
데이터베이스 연습 #314
SQLite CRUD 실습
"""
import sqlite3

def setup_314():
    conn = sqlite3.connect(':memory:')
    cursor = conn.cursor()
    cursor.execute('''CREATE TABLE items_314 (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL,
        value REAL,
        category TEXT
    )''')

    data = [
        [('item_314_0', 533.55, 'C'), ('item_314_1', 978.94, 'A'), ('item_314_2', 758.62, 'B'), ('item_314_3', 468.37, 'C'), ('item_314_4', 549.0, 'C'), ('item_314_5', 385.48, 'A'), ('item_314_6', 255.04, 'B'), ('item_314_7', 163.57, 'A')]
    ]
    cursor.executemany(
        'INSERT INTO items_314 (name, value, category) VALUES (?, ?, ?)',
        data
    )
    conn.commit()
    return conn

if __name__ == '__main__':
    conn = setup_314()
    cursor = conn.cursor()

    print(f"DB 연습 #314")
    cursor.execute('SELECT * FROM items_314')
    for row in cursor.fetchall():
        print(f"  {row}")

    cursor.execute('SELECT category, COUNT(*), AVG(value) FROM items_314 GROUP BY category')
    print("\n카테고리별 통계:")
    for row in cursor.fetchall():
        print(f"  {row[0]}: {row[1]}개, 평균 {row[2]:.2f}")

    conn.close()
